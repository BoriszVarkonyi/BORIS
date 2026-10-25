%% =========================================================
% RL V3 FIXED - CONTINUE TRAINING
%
% Kiindulás:
%   RL_SAC_V3_position_300ep.mat
%
% Folytatás:
%   +300 episode
%
% Végső fájl:
%   RL_SAC_V3_fixed_600ep.mat
%% =========================================================

clearvars;
clc;
close all;

%% =========================================================
% 1. ROBOT PARAMÉTEREK
%% =========================================================

run("init_pendulum.m");

% Training alatt mindig a vTrainCmd ág legyen aktív
trainingMode = 0;

%% =========================================================
% 2. MODELL
%% =========================================================

mdl = "pendulum_RL";
agentBlk = mdl + "/RL Agent";

load_system(mdl);

% Parallel simulation miatt a modell legyen elmentve
save_system(mdl);

%% =========================================================
% 3. MEGLÉVŐ AGENT BETÖLTÉSE
%% =========================================================

load("RL_SAC_V3_position_300ep.mat", ...
    "agent", ...
    "obsInfo", ...
    "actInfo", ...
    "Ts");

fprintf("\nMeglévő V3 agent betöltve.\n");

%% =========================================================
% 4. ENVIRONMENT
%% =========================================================

env = rlSimulinkEnv( ...
    mdl, ...
    agentBlk, ...
    obsInfo, ...
    actInfo);

env.ResetFcn = @resetRobot;

%% =========================================================
% 5. GPU
%% =========================================================

gpu = gpuDevice;

fprintf("GPU: %s\n", gpu.Name);
fprintf("Szabad GPU memória: %.2f GB\n", ...
    gpu.AvailableMemory / 1e9);

agent.UseGPUForLearning = true;

%% =========================================================
% 6. ENVIRONMENT ELLENŐRZÉS
%% =========================================================

validateEnvironment(env);

fprintf("Environment validálva.\n");

%% =========================================================
% 7. PARALLEL POOL
%% =========================================================

pool = gcp("nocreate");

if isempty(pool)

    fprintf("Parallel pool indítása 2 workerrel...\n");

    pool = parpool(2);

else

    fprintf("Már fut parallel pool %d workerrel.\n", ...
        pool.NumWorkers);

end

%% =========================================================
% 8. TRAINING OPTIONS
%% =========================================================

additionalEpisodes = 300;

maxSteps = 500;

trainOpts = rlTrainingOptions( ...
    MaxEpisodes = additionalEpisodes, ...
    MaxStepsPerEpisode = maxSteps, ...
    ScoreAveragingWindowLength = 20, ...
    StopTrainingCriteria = "None", ...
    Verbose = true, ...
    Plots = "training-progress");

trainOpts.UseParallel = true;

trainOpts.ParallelizationOptions.Mode = "async";

%% =========================================================
% 9. TRAINING INFO
%% =========================================================

fprintf("\n");
fprintf("=========================================\n");
fprintf(" RL V3 FIXED - CONTINUE TRAINING\n");
fprintf("=========================================\n");
fprintf("További episode-ok:       %d\n", ...
    additionalEpisodes);
fprintf("Max steps / episode:      %d\n", ...
    maxSteps);
fprintf("Agent sample time:        %.3f s\n", ...
    Ts);
fprintf("Parallel workers:         %d\n", ...
    pool.NumWorkers);
fprintf("GPU learning:             %d\n", ...
    agent.UseGPUForLearning);
fprintf("=========================================\n\n");

%% =========================================================
% 10. TRAINING FOLYTATÁSA
%% =========================================================

tic;

trainingStats_continue = train( ...
    agent, ...
    env, ...
    trainOpts);

trainingTime_continue = toc;

%% =========================================================
% 11. MENTÉS
%% =========================================================

saveFile = "RL_SAC_V3_fixed_600ep.mat";

save( ...
    saveFile, ...
    "agent", ...
    "trainingStats_continue", ...
    "obsInfo", ...
    "actInfo", ...
    "Ts", ...
    "trainingTime_continue");

%% =========================================================
% 12. EREDMÉNY
%% =========================================================

fprintf("\n");
fprintf("=========================================\n");
fprintf(" TRAINING FOLYTATÁS KÉSZ\n");
fprintf("=========================================\n");

fprintf("Training idő: %.1f s\n", ...
    trainingTime_continue);

fprintf("Training idő: %.2f perc\n", ...
    trainingTime_continue / 60);

fprintf("Mentve: %s\n", ...
    saveFile);

fprintf("=========================================\n");

%% =========================================================
% RESET FUNCTION
%% =========================================================

function in = resetRobot(in)

    %% Random kezdeti dőlésszög
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

    %% Random sebességparancs
    vTrainCmdEpisode = ...
        -0.15 + 0.30*rand;

    %% Episode változók
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

    %% FONTOS:
    % trainingMode = 0 -> vTrainCmd ág
    in = setVariable( ...
        in, ...
        "trainingMode", ...
        0);

end