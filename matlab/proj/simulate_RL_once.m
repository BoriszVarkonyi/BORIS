%% =========================================================
% TDK - Betanított RL controller egyszeri szimulációja
%
% A modellben már meglévő To Workspace blokkokat használja:
%
%   out.thetaFeedback
%   out.thetaDotLog
%   out.vLog
%   out.xWorldLog
%   out.yWorldLog
%   out.rewardLog
%   out.isDoneLog
%
% ==========================================================

clearvars;
clc;
close all;

%% =========================================================
% 1. ROBOT PARAMÉTEREK
% ==========================================================

run("init_pendulum.m");

%% =========================================================
% 2. MODELL ÉS BETANÍTOTT AGENT
% ==========================================================

mdl = "pendulum_RL";

load_system(mdl);

% A már betanított SAC agent betöltése
load("RL_first_test.mat", ...
    "agent", ...
    "obsInfo", ...
    "actInfo", ...
    "Ts");

fprintf("Betanított RL agent betöltve.\n");

%% =========================================================
% 3. TESZT BEÁLLÍTÁSA
%
% EZEKET KELL ÁTÍRNOD, HA MÁST AKARSZ TESZTELNI
% ==========================================================

theta0_deg = 5;       % kezdeti dőlés [deg]

vCmd_test = 0.0;      % sebesség referencia [m/s]

simTime = 10;         % szimuláció hossza [s]

%% =========================================================
% 4. KEZDETI ÁLLAPOT
% ==========================================================

q0_test = [
    0;                      % s [m]
    deg2rad(theta0_deg);     % theta [rad]
    0                       % psi [rad]
];

qd0_test = [
    0;                      % v [m/s]
    0;                      % thetaDot [rad/s]
    0                       % psiDot [rad/s]
];

%% =========================================================
% 5. AGENT TESZT ÜZEMMÓD
% ==========================================================

% Nem akarunk explorationt.
% A megtanult policy-t akarjuk látni.
agent.UseExplorationPolicy = false;

%% =========================================================
% 6. SIMULATION INPUT
% ==========================================================

simIn = Simulink.SimulationInput(mdl);

% Kezdeti állapotok
simIn = setVariable( ...
    simIn, ...
    "q0", ...
    q0_test);

simIn = setVariable( ...
    simIn, ...
    "qd0", ...
    qd0_test);

% Fix sebességparancs
simIn = setVariable( ...
    simIn, ...
    "vTrainCmd", ...
    vCmd_test);

% A képed alapján a Switch:
%
% trainingMode > 0.5  --> felső bemenet
% különben            --> alsó vTrainCmd bemenet
%
% Ezért a tesztnél 0-ra állítjuk, hogy biztosan
% a vTrainCmd_test menjen az RL agenthez.

simIn = setVariable( ...
    simIn, ...
    "trainingMode", ...
    0);

% Betanított agent
simIn = setVariable( ...
    simIn, ...
    "agent", ...
    agent);

% Szimuláció hossza
simIn = setModelParameter( ...
    simIn, ...
    "StopTime", ...
    num2str(simTime));

%% =========================================================
% 7. SZIMULÁCIÓ INDÍTÁSA
% ==========================================================

fprintf("\n");
fprintf("=========================================\n");
fprintf(" RL EGYSZERI SZIMULÁCIÓ\n");
fprintf("=========================================\n");

fprintf("Kezdeti dőlés:     %+6.2f deg\n", ...
    theta0_deg);

fprintf("Sebességparancs:   %+6.3f m/s\n", ...
    vCmd_test);

fprintf("Szimulációs idő:   %6.2f s\n", ...
    simTime);

fprintf("=========================================\n\n");

tic;

out = sim(simIn);

elapsedTime = toc;

fprintf("Szimuláció kész: %.2f s\n", ...
    elapsedTime);

%% =========================================================
% 8. ELLENŐRIZZÜK A TO WORKSPACE KIMENETEKET
% ==========================================================

availableOutputs = string(who(out));

requiredOutputs = [
    "thetaFeedback"
    "thetaDotLog"
    "vLog"
    "xWorldLog"
    "yWorldLog"
    "rewardLog"
    "isDoneLog"
];

missingOutputs = ...
    requiredOutputs(~ismember(requiredOutputs, ...
                              availableOutputs));

if ~isempty(missingOutputs)

    fprintf("\nHiányzó To Workspace változók:\n");

    disp(missingOutputs);

    fprintf("\nAz out objektumban jelenleg ezek vannak:\n");

    disp(availableOutputs);

    error( ...
        "Nem található minden szükséges To Workspace kimenet.");

end

%% =========================================================
% 9. TO WORKSPACE ADATOK KIOLVASÁSA
% ==========================================================

thetaLog    = out.thetaFeedback;
thetaDotLog = out.thetaDotLog;
vLog        = out.vLog;

xWorldLog   = out.xWorldLog;
yWorldLog   = out.yWorldLog;

rewardLog   = out.rewardLog;
isDoneLog   = out.isDoneLog;

%% =========================================================
% 10. TIMESERIES ADATOK KINYERÉSE
%
% A To Workspace blokkok nálad várhatóan Timeseries
% formátumban mentenek.
% ==========================================================

[tTheta, theta] = extractTimeData(thetaLog);

[tThetaDot, thetaDot] = ...
    extractTimeData(thetaDotLog);

[tV, v] = ...
    extractTimeData(vLog);

[tX, xWorld] = ...
    extractTimeData(xWorldLog);

[tY, yWorld] = ...
    extractTimeData(yWorldLog);

[tReward, reward] = ...
    extractTimeData(rewardLog);

[tDone, isDone] = ...
    extractTimeData(isDoneLog);

%% =========================================================
% 11. DŐLÉSSZÖG
% ==========================================================

thetaDeg = rad2deg(theta);

figure;

plot( ...
    tTheta, ...
    thetaDeg, ...
    "LineWidth", 1.5);

hold on;

yline(0, "--");

grid on;

xlabel("Idő [s]");
ylabel("\theta [deg]");

title(sprintf( ...
    "RL szabályozó - dőlésszög, \\theta_0 = %.1f°", ...
    theta0_deg));

%% =========================================================
% 12. DŐLÉSI SZÖGSEBESSÉG
% ==========================================================

figure;

plot( ...
    tThetaDot, ...
    thetaDot, ...
    "LineWidth", 1.5);

hold on;

yline(0, "--");

grid on;

xlabel("Idő [s]");
ylabel("\dot{\theta} [rad/s]");

title( ...
    "RL szabályozó - dőlési szögsebesség");

%% =========================================================
% 13. SEBESSÉGKÖVETÉS
% ==========================================================

figure;

plot( ...
    tV, ...
    v, ...
    "LineWidth", 1.5);

hold on;

% Mivel a referencia most konstans, nem kell külön logolni.
yline( ...
    vCmd_test, ...
    "--", ...
    sprintf("v_{cmd} = %.2f m/s", vCmd_test), ...
    "LineWidth", 1.2);

grid on;

xlabel("Idő [s]");
ylabel("v [m/s]");

title("RL szabályozó - sebességkövetés");

legend( ...
    "v", ...
    "v_{cmd}", ...
    "Location", "best");

%% =========================================================
% 14. VILÁGKOORDINÁTÁS PÁLYA
% ==========================================================

% Biztos ami biztos: ha a két logger mintaszáma
% minimálisan eltérne, a közös részt használjuk.

Nxy = min( ...
    numel(xWorld), ...
    numel(yWorld));

xPlot = xWorld(1:Nxy);
yPlot = yWorld(1:Nxy);

figure;

plot( ...
    xPlot, ...
    yPlot, ...
    "LineWidth", 1.6);

hold on;

plot( ...
    xPlot(1), ...
    yPlot(1), ...
    "o", ...
    "MarkerSize", 8, ...
    "LineWidth", 1.5);

plot( ...
    xPlot(end), ...
    yPlot(end), ...
    "x", ...
    "MarkerSize", 10, ...
    "LineWidth", 2);

axis equal;
grid on;

xlabel("x [m]");
ylabel("y [m]");

title( ...
    "RL szabályozó - világkoordinátás pálya");

legend( ...
    "Pálya", ...
    "Kezdőpont", ...
    "Végpont", ...
    "Location", "best");

%% =========================================================
% 15. REWARD
% ==========================================================

figure;

plot( ...
    tReward, ...
    reward, ...
    "LineWidth", 1.3);

grid on;

xlabel("Idő [s]");
ylabel("Reward");

title( ...
    "RL szabályozó - pillanatnyi reward");

%% =========================================================
% 16. NUMERIKUS EREDMÉNYEK
% ==========================================================

maxTheta = ...
    max(abs(thetaDeg));

finalTheta = ...
    thetaDeg(end);

maxThetaDot = ...
    max(abs(thetaDot));

finalVelocity = ...
    v(end);

velocityError = ...
    vCmd_test - v;

velocityRMSE = ...
    sqrt(mean(velocityError.^2));

thetaRMSE = ...
    sqrt(mean(thetaDeg.^2));

totalReward = ...
    sum(reward);

meanReward = ...
    mean(reward);

fell = ...
    any(logical(isDone));

%% =========================================================
% 17. EREDMÉNY KIÍRÁSA
% ==========================================================

fprintf("\n");
fprintf("=========================================\n");
fprintf(" RL SZIMULÁCIÓ EREDMÉNYE\n");
fprintf("=========================================\n");

fprintf( ...
    "Kezdeti theta:          %+8.3f deg\n", ...
    theta0_deg);

fprintf( ...
    "Max |theta|:            %8.3f deg\n", ...
    maxTheta);

fprintf( ...
    "Végső theta:            %+8.3f deg\n", ...
    finalTheta);

fprintf( ...
    "Theta RMSE:             %8.3f deg\n", ...
    thetaRMSE);

fprintf( ...
    "Max |thetaDot|:         %8.3f rad/s\n", ...
    maxThetaDot);

fprintf( ...
    "vCmd:                   %+8.4f m/s\n", ...
    vCmd_test);

fprintf( ...
    "Végső sebesség:         %+8.4f m/s\n", ...
    finalVelocity);

fprintf( ...
    "Sebesség RMSE:          %8.4f m/s\n", ...
    velocityRMSE);

fprintf( ...
    "Végső x koordináta:     %+8.4f m\n", ...
    xWorld(end));

fprintf( ...
    "Végső y koordináta:     %+8.4f m\n", ...
    yWorld(end));

fprintf( ...
    "Összes reward:          %8.2f\n", ...
    totalReward);

fprintf( ...
    "Átlag reward / step:    %8.4f\n", ...
    meanReward);

fprintf( ...
    "Elesett:                %s\n", ...
    string(fell));

fprintf( ...
    "Szimuláció futási idő:  %8.2f s\n", ...
    elapsedTime);

fprintf("=========================================\n");

%% =========================================================
% HELPER FUNCTION
%
% To Workspace blokk adatainak egységes feldolgozása.
%
% Timeseries esetén:
%   Time + Data
%
% Structure With Time esetén:
%   time + signals.values
%
% ==========================================================

function [t, data] = extractTimeData(loggedSignal)

    if isa(loggedSignal, "timeseries")

        t = loggedSignal.Time;
        data = loggedSignal.Data;

    elseif isstruct(loggedSignal) && ...
           isfield(loggedSignal, "time") && ...
           isfield(loggedSignal, "signals")

        t = loggedSignal.time;
        data = loggedSignal.signals.values;

    elseif istimetable(loggedSignal)

        t = seconds( ...
            loggedSignal.Properties.RowTimes - ...
            loggedSignal.Properties.RowTimes(1));

        data = loggedSignal.Variables;

    else

        error( ...
            "Ismeretlen To Workspace formátum: %s", ...
            class(loggedSignal));

    end

    t = squeeze(t);
    data = squeeze(data);

end