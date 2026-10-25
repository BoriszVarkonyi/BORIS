%% =========================================================
% RL V3 - DETERMINISZTIKUS VALIDÁCIÓ
%
% Trained agent:
%   RL_SAC_V3_position_500ep.mat
%
% Tesztek:
%   vCmd =  0.00 m/s
%   vCmd = +0.10 m/s
%   vCmd = -0.10 m/s
%
% Kezdeti dőlésszög:
%   theta0 = +5 deg
%
% Szimuláció:
%   10 s
%% =========================================================

clearvars;
clc;
close all;

%% =========================================================
% 1. PARAMÉTEREK
%% =========================================================

run("init_pendulum.m");

mdl = "pendulum_RL";

load_system(mdl);

%% =========================================================
% 2. TRAINED AGENT BETÖLTÉSE
%% =========================================================

load("RL_SAC_V3_position_500ep.mat", ...
    "agent", ...
    "obsInfo", ...
    "actInfo", ...
    "Ts");

% Validációnál ne exploráljon
agent.UseExplorationPolicy = false;

%% =========================================================
% 3. TESZTPARAMÉTEREK
%% =========================================================

theta0_deg = 5;

simTime = 10;

vCmdTests = [
     0.00
     0.10
    -0.10
];

%% Kezdeti állapot
q0_test = [
    0;                          % s
    deg2rad(theta0_deg);        % theta
    0                           % psi
];

qd0_test = [
    0;                          % v
    0;                          % thetaDot
    0                           % psiDot
];

%% Eredménytábla előkészítés
nTests = length(vCmdTests);

results = table( ...
    zeros(nTests,1), ...
    zeros(nTests,1), ...
    zeros(nTests,1), ...
    zeros(nTests,1), ...
    zeros(nTests,1), ...
    zeros(nTests,1), ...
    zeros(nTests,1), ...
    zeros(nTests,1), ...
    zeros(nTests,1), ...
    false(nTests,1), ...
    'VariableNames', { ...
        'vCmd', ...
        'MaxThetaDeg', ...
        'FinalThetaDeg', ...
        'ThetaRMSEDeg', ...
        'FinalVelocity', ...
        'VelocityRMSE', ...
        'FinalPosition', ...
        'FinalPositionError', ...
        'PositionRMSE', ...
        'Fell'});

%% =========================================================
% 4. TESZTEK
%% =========================================================

for k = 1:nTests

    vCmd_test = vCmdTests(k);

    fprintf("\n");
    fprintf("========================================\n");
    fprintf("TEST %d / %d\n", k, nTests);
    fprintf("theta0 = %.1f deg\n", theta0_deg);
    fprintf("vCmd   = %.3f m/s\n", vCmd_test);
    fprintf("========================================\n");

    %% -----------------------------------------------------
    % SimulationInput
    %% -----------------------------------------------------

    simIn = Simulink.SimulationInput(mdl);

    simIn = setVariable( ...
        simIn, ...
        "q0", ...
        q0_test);

    simIn = setVariable( ...
        simIn, ...
        "qd0", ...
        qd0_test);

    simIn = setVariable( ...
        simIn, ...
        "vTrainCmd", ...
        vCmd_test);

    % A jelenlegi Switch alapján:
    %
    % trainingMode = 0
    % -> alsó vTrainCmd ág
    %
    simIn = setVariable( ...
        simIn, ...
        "trainingMode", ...
        0);

    simIn = setVariable( ...
        simIn, ...
        "agent", ...
        agent);

    simIn = setModelParameter( ...
        simIn, ...
        "StopTime", ...
        num2str(simTime));

    %% -----------------------------------------------------
    % Szimuláció
    %% -----------------------------------------------------

    out = sim(simIn);

    %% =====================================================
    % 5. ADATOK KIOLVASÁSA
    %% =====================================================

    [tTheta, theta] = ...
        extractTimeData(out.thetaFeedback);

    [tThetaDot, thetaDot] = ...
        extractTimeData(out.thetaDotLog);

    [tV, v] = ...
        extractTimeData(out.vLog);

    [tS, s] = ...
        extractTimeData(out.sLog);

    [tTau, tauCommon] = ...
        extractTimeData(out.tauCommonLog);

    [tReward, reward] = ...
        extractTimeData(out.rewardLog);

    [~, isDone] = ...
        extractTimeData(out.isDoneLog);

    %% Oszlopvektorok
    tTheta = tTheta(:);
    theta = theta(:);

    tThetaDot = tThetaDot(:);
    thetaDot = thetaDot(:);

    tV = tV(:);
    v = v(:);

    tS = tS(:);
    s = s(:);

    tTau = tTau(:);
    tauCommon = tauCommon(:);

    tReward = tReward(:);
    reward = reward(:);

    isDone = isDone(:);

    %% =====================================================
    % 6. REFERENCIAJELEK
    %% =====================================================

    % Konstans sebességparancs:
    %
    % sRef(t) = integral(vCmd dt)
    %
    % sRef(0) = 0

    sRef = vCmd_test .* tS;

    vRef = vCmd_test .* ones(size(tV));

    %% =====================================================
    % 7. HIBÁK
    %% =====================================================

    thetaDeg = rad2deg(theta);

    velocityError = ...
        vRef - v;

    positionError = ...
        sRef - s;

    %% =====================================================
    % 8. MÉRŐSZÁMOK
    %% =====================================================

    maxTheta = ...
        max(abs(thetaDeg));

    finalTheta = ...
        thetaDeg(end);

    thetaRMSE = ...
        sqrt(mean(thetaDeg.^2));

    finalV = ...
        v(end);

    velocityRMSE = ...
        sqrt(mean(velocityError.^2));

    finalS = ...
        s(end);

    finalSRef = ...
        sRef(end);

    finalPositionError = ...
        finalSRef - finalS;

    positionRMSE = ...
        sqrt(mean(positionError.^2));

    maxTau = ...
        max(abs(tauCommon));

    meanReward = ...
        mean(reward);

    totalReward = ...
        sum(reward);

    fell = ...
        any(isDone > 0.5);

    %% =====================================================
    % 9. TERMINÁL KIÍRÁS
    %% =====================================================

    fprintf("\n--- EREDMÉNY ---\n");

    fprintf( ...
        "Max |theta|:          %.4f deg\n", ...
        maxTheta);

    fprintf( ...
        "Final theta:          %.4f deg\n", ...
        finalTheta);

    fprintf( ...
        "Theta RMSE:           %.4f deg\n", ...
        thetaRMSE);

    fprintf( ...
        "Final velocity:       %.4f m/s\n", ...
        finalV);

    fprintf( ...
        "Velocity target:      %.4f m/s\n", ...
        vCmd_test);

    fprintf( ...
        "Velocity RMSE:        %.4f m/s\n", ...
        velocityRMSE);

    fprintf( ...
        "Final position:       %.4f m\n", ...
        finalS);

    fprintf( ...
        "Final sRef:           %.4f m\n", ...
        finalSRef);

    fprintf( ...
        "Final position error: %.4f m\n", ...
        finalPositionError);

    fprintf( ...
        "Position RMSE:        %.4f m\n", ...
        positionRMSE);

    fprintf( ...
        "Max |tauCommon|:      %.4f Nm\n", ...
        maxTau);

    fprintf( ...
        "Mean reward:          %.4f\n", ...
        meanReward);

    fprintf( ...
        "Total reward:         %.2f\n", ...
        totalReward);

    fprintf( ...
        "Fell:                 %d\n", ...
        fell);

    %% =====================================================
    % 10. RESULTS TÁBLA
    %% =====================================================

    results.vCmd(k) = ...
        vCmd_test;

    results.MaxThetaDeg(k) = ...
        maxTheta;

    results.FinalThetaDeg(k) = ...
        finalTheta;

    results.ThetaRMSEDeg(k) = ...
        thetaRMSE;

    results.FinalVelocity(k) = ...
        finalV;

    results.VelocityRMSE(k) = ...
        velocityRMSE;

    results.FinalPosition(k) = ...
        finalS;

    results.FinalPositionError(k) = ...
        finalPositionError;

    results.PositionRMSE(k) = ...
        positionRMSE;

    results.Fell(k) = ...
        fell;

    %% =====================================================
    % 11. ÁBRA
    %% =====================================================

    fig = figure( ...
        "Name", ...
        sprintf( ...
            "RL V3 - vCmd = %.2f m/s", ...
            vCmd_test), ...
        "NumberTitle", ...
        "off");

    tl = tiledlayout( ...
        fig, ...
        3, ...
        2);

    tl.TileSpacing = "compact";
    tl.Padding = "compact";

    %% -----------------------------------------------------
    % Dőlésszög
    %% -----------------------------------------------------

    nexttile;

    plot( ...
        tTheta, ...
        thetaDeg, ...
        "LineWidth", ...
        1.3);

    hold on;

    yline(0, "--");

    grid on;

    xlabel("t [s]");
    ylabel("\theta [deg]");

    title("Dőlésszög");

    %% -----------------------------------------------------
    % Dőlési szögsebesség
    %% -----------------------------------------------------

    nexttile;

    plot( ...
        tThetaDot, ...
        thetaDot, ...
        "LineWidth", ...
        1.3);

    hold on;

    yline(0, "--");

    grid on;

    xlabel("t [s]");

    ylabel( ...
        "$\dot{\theta}$ [rad/s]", ...
        "Interpreter", ...
        "latex");

    title("Dőlési szögsebesség");

    %% -----------------------------------------------------
    % Sebesség
    %% -----------------------------------------------------

    nexttile;

    plot( ...
        tV, ...
        v, ...
        "LineWidth", ...
        1.3);

    hold on;

    plot( ...
        tV, ...
        vRef, ...
        "--", ...
        "LineWidth", ...
        1.2);

    grid on;

    xlabel("t [s]");
    ylabel("v [m/s]");

    title("Sebességkövetés");

    legend( ...
        "v", ...
        "v_{cmd}", ...
        "Location", ...
        "best");

    %% -----------------------------------------------------
    % Pozíció
    %% -----------------------------------------------------

    nexttile;

    plot( ...
        tS, ...
        s, ...
        "LineWidth", ...
        1.3);

    hold on;

    plot( ...
        tS, ...
        sRef, ...
        "--", ...
        "LineWidth", ...
        1.2);

    grid on;

    xlabel("t [s]");
    ylabel("s [m]");

    title("Pozíciókövetés");

    legend( ...
        "s", ...
        "s_{ref}", ...
        "Location", ...
        "best");

    %% -----------------------------------------------------
    % Közös nyomaték
    %% -----------------------------------------------------

    nexttile;

    plot( ...
        tTau, ...
        tauCommon, ...
        "LineWidth", ...
        1.3);

    hold on;

    yline(2, "--");
    yline(-2, "--");

    grid on;

    xlabel("t [s]");
    ylabel("\tau_c [Nm]");

    title("Közös hajtónyomaték");

    %% -----------------------------------------------------
    % Reward
    %% -----------------------------------------------------

    nexttile;

    plot( ...
        tReward, ...
        reward, ...
        "LineWidth", ...
        1.3);

    grid on;

    xlabel("t [s]");
    ylabel("Reward");

    title("Pillanatnyi reward");

    %% Főcím
    title( ...
        tl, ...
        sprintf( ...
            "RL V3 | theta0 = %.1f deg | vCmd = %.2f m/s", ...
            theta0_deg, ...
            vCmd_test));

end

%% =========================================================
% 12. ÖSSZESÍTÉS
%% =========================================================

fprintf("\n\n");
fprintf("=============================================\n");
fprintf("RL V3 VALIDÁCIÓ ÖSSZESÍTÉS\n");
fprintf("=============================================\n\n");

disp(results);

%% Mentés
save( ...
    "RL_V3_validation_results.mat", ...
    "results");

writetable( ...
    results, ...
    "RL_V3_validation_results.csv");

fprintf("\nEredmények mentve:\n");
fprintf("RL_V3_validation_results.mat\n");
fprintf("RL_V3_validation_results.csv\n");


%% =========================================================
% HELPER FUNCTION
%% =========================================================

function [t, data] = extractTimeData(signal)

    %% Timeseries
    if isa(signal, "timeseries")

        t = signal.Time;
        data = signal.Data;

    %% Structure with time
    elseif isstruct(signal)

        if isfield(signal, "time")

            t = signal.time;

        elseif isfield(signal, "Time")

            t = signal.Time;

        else

            error( ...
                "A struktúrában nincs idővektor.");

        end

        if isfield(signal, "signals")

            data = signal.signals.values;

        elseif isfield(signal, "Data")

            data = signal.Data;

        else

            error( ...
                "A struktúrában nincs adatmező.");

        end

    %% Timetable
    elseif istimetable(signal)

        t = ...
            seconds(signal.Time - signal.Time(1));

        data = ...
            signal.Variables;

    %% Egyéb
    else

        error( ...
            "Ismeretlen To Workspace adattípus: %s", ...
            class(signal));

    end

    t = squeeze(t);
    data = squeeze(data);

end