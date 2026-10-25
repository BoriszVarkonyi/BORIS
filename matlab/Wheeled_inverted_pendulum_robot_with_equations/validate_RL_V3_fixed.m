%% =========================================================
% RL V3 FIXED - DETERMINISZTIKUS VALIDÁCIÓ
%
% Agent:
%   RL_SAC_V3_fixed_600ep.mat
%
% Tesztek:
%   vCmd =  0.00 m/s
%   vCmd = +0.10 m/s
%   vCmd = -0.10 m/s
%
% theta0 = +5 deg
% simTime = 10 s
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
% 2. BETANÍTOTT AGENT
%% =========================================================

load("RL_SAC_V3_fixed_600ep.mat", ...
    "agent", ...
    "obsInfo", ...
    "actInfo", ...
    "Ts");

% Validáció közben nincs exploration
agent.UseExplorationPolicy = false;

fprintf("Agent betöltve: RL_SAC_V3_fixed_600ep.mat\n");

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

nTests = length(vCmdTests);

%% Kezdeti állapot

q0_test = [
    0;
    deg2rad(theta0_deg);
    0
];

qd0_test = [
    0;
    0;
    0
];

%% =========================================================
% 4. EREDMÉNYTÁBLA
%% =========================================================

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
    zeros(nTests,1), ...
    zeros(nTests,1), ...
    false(nTests,1), ...
    'VariableNames', { ...
        'vCmd', ...
        'MaxThetaDeg', ...
        'FinalThetaDeg', ...
        'ThetaRMSEDeg', ...
        'FinalVelocity', ...
        'VelocityErrorFinal', ...
        'VelocityRMSE', ...
        'FinalPosition', ...
        'FinalPositionRef', ...
        'FinalPositionError', ...
        'PositionRMSE', ...
        'Fell'});

%% =========================================================
% 5. TESZTEK
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

    % Fix sebességparancs
    simIn = setVariable( ...
        simIn, ...
        "vTrainCmd", ...
        vCmd_test);

    % FONTOS:
    % 0 -> vTrainCmd ág
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

    %% -----------------------------------------------------
    % Szimuláció
    %% -----------------------------------------------------

    out = sim(simIn);

    %% =====================================================
    % 6. LOGOK KIOLVASÁSA
    %% =====================================================

    [tTheta, theta] = ...
        extractTimeData(out.thetaFeedback);

    [tThetaDot, thetaDot] = ...
        extractTimeData(out.thetaDotLog);

    [tV, v] = ...
        extractTimeData(out.vLog);

    [tS, s] = ...
        extractTimeData(out.sLog);

    [tSRef, sRefActual] = ...
        extractTimeData(out.sRefLog);

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

    tSRef = tSRef(:);
    sRefActual = sRefActual(:);

    tTau = tTau(:);
    tauCommon = tauCommon(:);

    tReward = tReward(:);
    reward = reward(:);

    isDone = isDone(:);

    %% =====================================================
    % 7. REFERENCIAJELEK
    %% =====================================================

    vRef = vCmd_test .* ones(size(tV));

    % Ha az sRef logger rendesen működik,
    % ezt használjuk a tényleges modellből.
    %
    % Elméletileg:
    %
    % sRef = vCmd * t

    sRef = sRefActual;

    %% =====================================================
    % 8. HIBÁK
    %% =====================================================

    thetaDeg = rad2deg(theta);

    velocityError = ...
        vRef - v;

    % s és sRef időpontjai várhatóan megegyeznek,
    % de biztosra megyünk.
    if numel(sRef) == numel(s)

        positionError = ...
            sRef - s;

        sRefForPlot = sRef;

    else

        sRefForPlot = interp1( ...
            tSRef, ...
            sRef, ...
            tS, ...
            "linear", ...
            "extrap");

        positionError = ...
            sRefForPlot - s;

    end

    %% =====================================================
    % 9. MÉRŐSZÁMOK
    %% =====================================================

    maxTheta = ...
        max(abs(thetaDeg));

    finalTheta = ...
        thetaDeg(end);

    thetaRMSE = ...
        sqrt(mean(thetaDeg.^2));

    finalV = ...
        v(end);

    finalVelocityError = ...
        vCmd_test - finalV;

    velocityRMSE = ...
        sqrt(mean(velocityError.^2));

    finalS = ...
        s(end);

    finalSRef = ...
        sRefForPlot(end);

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
    % 10. TERMINÁL
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

    fprintf("\n");

    fprintf( ...
        "Final velocity:       %.4f m/s\n", ...
        finalV);

    fprintf( ...
        "Velocity target:      %.4f m/s\n", ...
        vCmd_test);

    fprintf( ...
        "Final velocity error: %.4f m/s\n", ...
        finalVelocityError);

    fprintf( ...
        "Velocity RMSE:        %.4f m/s\n", ...
        velocityRMSE);

    fprintf("\n");

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

    fprintf("\n");

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
    % 11. RESULTS TÁBLA
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

    results.VelocityErrorFinal(k) = ...
        finalVelocityError;

    results.VelocityRMSE(k) = ...
        velocityRMSE;

    results.FinalPosition(k) = ...
        finalS;

    results.FinalPositionRef(k) = ...
        finalSRef;

    results.FinalPositionError(k) = ...
        finalPositionError;

    results.PositionRMSE(k) = ...
        positionRMSE;

    results.Fell(k) = ...
        fell;

    %% =====================================================
    % 12. ÁBRÁK
    %% =====================================================

    fig = figure( ...
        "Name", ...
        sprintf( ...
            "RL V3 FIXED - vCmd = %.2f m/s", ...
            vCmd_test), ...
        "NumberTitle", ...
        "off");

    tl = tiledlayout(fig, 3, 2);

    tl.TileSpacing = "compact";
    tl.Padding = "compact";

    %% -----------------------------------------------------
    % 1. Dőlésszög
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
    % 2. Dőlési szögsebesség
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
    % 3. Sebesség
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
    % 4. Pozíció
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
        sRefForPlot, ...
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
    % 5. Közös nyomaték
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
    % 6. Reward
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
        "RL V3 FIXED | theta0 = %.1f deg | vCmd = %.2f m/s", ...
        theta0_deg, ...
        vCmd_test));

end

%% =========================================================
% 13. ÖSSZESÍTÉS
%% =========================================================

fprintf("\n\n");
fprintf("=================================================\n");
fprintf(" RL V3 FIXED VALIDÁCIÓ ÖSSZESÍTÉS\n");
fprintf("=================================================\n\n");

disp(results);

%% =========================================================
% 14. MENTÉS
%% =========================================================

save( ...
    "RL_V3_fixed_validation_results.mat", ...
    "results");

writetable( ...
    results, ...
    "RL_V3_fixed_validation_results.csv");

fprintf("\nEredmények mentve:\n");
fprintf("RL_V3_fixed_validation_results.mat\n");
fprintf("RL_V3_fixed_validation_results.csv\n");

%% =========================================================
% HELPER FUNCTION
%% =========================================================

function [t, data] = extractTimeData(signal)

    if isa(signal, "timeseries")

        t = signal.Time;
        data = signal.Data;

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

    elseif istimetable(signal)

        t = seconds( ...
            signal.Time - signal.Time(1));

        data = signal.Variables;

    else

        error( ...
            "Ismeretlen To Workspace adattípus: %s", ...
            class(signal));

    end

    t = squeeze(t);
    data = squeeze(data);

end