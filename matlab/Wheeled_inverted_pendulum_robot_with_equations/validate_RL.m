%% =========================================================
% TDK - Betanított RL szabályozó validációja
% ==========================================================

clearvars;
clc;
close all;

%% ---------------------------------------------------------
% 1. Robotparaméterek
% ----------------------------------------------------------

run("init_pendulum.m");

%% ---------------------------------------------------------
% 2. Modell és betanított agent
% ---------------------------------------------------------

mdl = "pendulum_RL";
agentBlk = mdl + "/RL Agent";

load_system(mdl);

% A tréning után elmentett állomány
load("RL_first_test.mat", ...
    "agent", ...
    "obsInfo", ...
    "actInfo", ...
    "Ts");

fprintf("Betanított agent betöltve.\n");

%% ---------------------------------------------------------
% 3. Exploration kikapcsolása
% ---------------------------------------------------------

% Validációnál nem akarjuk, hogy az agent véletlenszerűen
% exploráljon. A megtanult policy-t akarjuk vizsgálni.
agent.UseExplorationPolicy = false;

%% ---------------------------------------------------------
% 4. RL environment létrehozása
% ---------------------------------------------------------

env = rlSimulinkEnv( ...
    mdl, ...
    agentBlk, ...
    obsInfo, ...
    actInfo);

%% ---------------------------------------------------------
% 5. Random validáció
%
% Ugyanazon tartományban tesztelünk, amelyen tanítottuk:
%
% theta0   = -5 ... +5 fok
% vCmd     = -0.15 ... +0.15 m/s
% ---------------------------------------------------------

env.ResetFcn = @resetValidationRandom;

numberOfTests = 20;

episodeTime = 10;                  % [s]
maxSteps = round(episodeTime/Ts);  % 500 step

simOpts = rlSimulationOptions( ...
    MaxSteps = maxSteps, ...
    NumSimulations = numberOfTests);

fprintf("\n");
fprintf("=========================================\n");
fprintf(" RANDOM RL VALIDÁCIÓ INDUL\n");
fprintf(" %d teszt, egyenként max. %.1f s\n", ...
    numberOfTests, episodeTime);
fprintf("=========================================\n\n");

%% ---------------------------------------------------------
% 6. Szimuláció
% ---------------------------------------------------------

tic;

experience = sim(env, agent, simOpts);

validationTime = toc;

%% ---------------------------------------------------------
% 7. Eredmények feldolgozása
% ---------------------------------------------------------

episodeNumber = (1:numberOfTests)';

steps       = zeros(numberOfTests,1);
totalReward = zeros(numberOfTests,1);
rewardPerStep = zeros(numberOfTests,1);
fell        = false(numberOfTests,1);
survived    = false(numberOfTests,1);

for k = 1:numberOfTests

    % Reward idősor
    rewardData = double(experience(k).Reward.Data(:));

    % IsDone idősor
    doneData = logical(experience(k).IsDone.Data(:));

    % Episode hossza
    steps(k) = numel(rewardData);

    % Összes reward
    totalReward(k) = sum(rewardData);

    % Átlagos reward egy timestepre
    rewardPerStep(k) = mean(rewardData);

    % Elesett-e?
    if ~isempty(doneData)
        fell(k) = doneData(end);
    end

    % Végigment-e a teljes 10 másodpercen?
    survived(k) = steps(k) >= maxSteps;

end

%% ---------------------------------------------------------
% 8. Táblázat
% ---------------------------------------------------------

results = table( ...
    episodeNumber, ...
    steps, ...
    totalReward, ...
    rewardPerStep, ...
    fell, ...
    survived, ...
    'VariableNames', { ...
        'Test', ...
        'Steps', ...
        'TotalReward', ...
        'RewardPerStep', ...
        'Fell', ...
        'Survived10s'});

disp(results);

%% ---------------------------------------------------------
% 9. Összesített statisztika
% ---------------------------------------------------------

successCount = sum(survived);

successRate = ...
    100 * successCount / numberOfTests;

fprintf("\n");
fprintf("=========================================\n");
fprintf(" RL VALIDÁCIÓ EREDMÉNYE\n");
fprintf("=========================================\n");

fprintf("Tesztekszám:              %d\n", ...
    numberOfTests);

fprintf("10 s túlélés:             %d / %d\n", ...
    successCount, numberOfTests);

fprintf("Sikerarány:               %.1f %%\n", ...
    successRate);

fprintf("Átlagos episode hossz:    %.1f step\n", ...
    mean(steps));

fprintf("Átlagos túlélési idő:     %.2f s\n", ...
    mean(steps)*Ts);

fprintf("Átlagos total reward:     %.2f\n", ...
    mean(totalReward));

fprintf("Átlagos reward / step:    %.4f\n", ...
    mean(rewardPerStep));

fprintf("Legrosszabb total reward: %.2f\n", ...
    min(totalReward));

fprintf("Legjobb total reward:     %.2f\n", ...
    max(totalReward));

fprintf("Validáció futási idő:     %.1f s\n", ...
    validationTime);

fprintf("=========================================\n");

%% ---------------------------------------------------------
% 10. Episode hossz grafikon
% ---------------------------------------------------------

figure;

plot(episodeNumber, steps, '-o', ...
    'LineWidth', 1.2);

hold on;

yline(maxSteps, '--', ...
    sprintf('Maximum = %d step', maxSteps));

grid on;

xlabel('Teszt sorszáma');
ylabel('Episode steps');

title('RL szabályozó - túlélési idő');

ylim([0 maxSteps*1.05]);

%% ---------------------------------------------------------
% 11. Reward / step grafikon
% ---------------------------------------------------------

figure;

plot(episodeNumber, rewardPerStep, '-o', ...
    'LineWidth', 1.2);

grid on;

xlabel('Teszt sorszáma');
ylabel('Reward / step');

title('RL szabályozó - átlagos reward időlépésenként');

%% ---------------------------------------------------------
% 12. Validáció mentése
% ---------------------------------------------------------

timestamp = string(datetime("now", ...
    "Format","yyyyMMdd_HHmmss"));

filename = ...
    "RL_validation_" + timestamp + ".mat";

save(filename, ...
    "results", ...
    "experience", ...
    "successRate", ...
    "validationTime");

fprintf("\nEredmények mentve: %s\n", filename);


%% =========================================================
% RANDOM VALIDÁCIÓS RESET FUNCTION
% ==========================================================

function in = resetValidationRandom(in)

    %% Véletlen kezdeti dőlés
    theta0 = deg2rad(-5 + 10*rand);

    q0episode = [
        0;          % s
        theta0;     % theta
        0           % psi
    ];

    %% Nulla kezdeti sebesség

    qd0episode = [
        0;          % v
        0;          % thetaDot
        0           % psiDot
    ];

    %% Véletlen sebességparancs

    vTrainCmdEpisode = ...
        -0.15 + 0.30*rand;

    %% Simuláció változóinak beállítása

    in = setVariable( ...
        in, ...
        "q0", ...
        q0episode);

    in = setVariable( ...
        in, ...
        "qd0", ...
        qd0episode);

    in = setVariable( ...
        in, ...
        "vTrainCmd", ...
        vTrainCmdEpisode);

end