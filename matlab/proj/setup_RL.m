%% =========================================================
%  TDK - RL environment és SAC agent inicializálása
% ==========================================================

clearvars;
clc;

%% 1. A robot paramétereinek betöltése

run("init_pendulum.m");

%% 2. Simulink modell

mdl = "pendulum_RL";

% Az RL Agent blokk teljes elérési útja.
% Ha a blokk neve nem pontosan "RL Agent", ezt módosítani kell.
agentBlk = mdl + "/RL Agent";

load_system(mdl);

%% 3. RL mintavételi idő

Ts = 0.02;        % [s] -> 50 Hz

%% 4. Observation space

obsInfo = rlNumericSpec([4 1]);

obsInfo.Name = "robotObservations";

obsInfo.Description = ...
    "vCmd, v, theta, thetaDot";

%% 5. Action space

actInfo = rlNumericSpec([1 1]);

actInfo.Name = "tauCommon";

actInfo.LowerLimit = -tauMax;
actInfo.UpperLimit =  tauMax;

%% 6. Simulink RL environment létrehozása

env = rlSimulinkEnv( ...
    mdl, ...
    agentBlk, ...
    obsInfo, ...
    actInfo);

%% 7. Reset függvény hozzárendelése

env.ResetFcn = @resetRobot;

%% 8. SAC agent létrehozása

agent = rlSACAgent(obsInfo, actInfo);

% A controller 20 ms-onként hoz új döntést.
agent.SampleTime = Ts;

% Jövőbeli rewardok fontossága.
agent.AgentOptions.DiscountFactor = 0.995;

% GPU használata a neurális hálók tanításához
agent.UseGPUForLearning = true;

%% 9. Environment ellenőrzése

validateEnvironment(env);

disp("RL environment sikeresen létrehozva.");
disp(agent);


%% =========================================================
% RESET FUNCTION
% Minden episode előtt lefut.
% ==========================================================

function in = resetRobot(in)

    %% Véletlen kezdeti dőlésszög
    theta0 = deg2rad(-5 + 10*rand);

    q0episode = [
        0;          % s
        theta0;     % theta
        0           % psi
    ];

    %% Kezdeti sebességek

    qd0episode = [
        0;          % v
        0;          % thetaDot
        0           % psiDot
    ];

    %% Véletlen sebességparancs

    vTrainCmdEpisode = -0.15 + 0.30*rand;

    %% Az adott szimuláció változóinak felülírása

    in = setVariable(in,"q0",q0episode);
    in = setVariable(in,"qd0",qd0episode);
    in = setVariable(in,"vTrainCmd",vTrainCmdEpisode);

end

%% =========================================================
% ELSŐ SAC TESZTTRÉNING
% ==========================================================

% Egy episode maximális szimulációs ideje
episodeTime = 10;          % [s]

% Az agent 0.02 másodpercenként dönt
% -> 10 s / 0.02 s = 500 decision / episode
maxSteps = round(episodeTime / Ts);

%% Training beállítások

pool = gcp('nocreate');

if isempty(pool)
    parpool(2);
end

trainOpts = rlTrainingOptions( ...
    MaxEpisodes = 300, ...
    MaxStepsPerEpisode = 500, ...
    ScoreAveragingWindowLength = 20, ...
    StopTrainingCriteria = "None", ...
    Verbose = true, ...
    Plots = "training-progress", ...
    UseParallel = true);

trainOpts.ParallelizationOptions.Mode = "async";

%% Tréning indítása

tic

trainingStats = train(agent, env, trainOpts);

elapsedTime = toc;

fprintf("Training idő: %.1f s\n", elapsedTime);

%% Eredmény mentése

save("RL_first_test.mat", ...
    "agent", ...
    "trainingStats", ...
    "obsInfo", ...
    "actInfo", ...
    "Ts");

disp("Első RL teszttréning befejezve.");