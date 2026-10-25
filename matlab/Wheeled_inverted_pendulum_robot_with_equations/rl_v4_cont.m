%% =========================================================
% RL V4 - CONTINUE TRAINING
%
% Kiindulás:
%   RL_SAC_V4_positionW1_500ep.mat
%
% Folytatás:
%   +200 episode
%
% Mentés:
%   RL_SAC_V4_positionW1_700ep.mat
%% =========================================================

clearvars;
clc;
close all;

%% =========================================================
% 1. ROBOT PARAMÉTEREK
%% =========================================================

run("init_pendulum.m");

% Training alatt a vTrainCmd ág legyen aktív
trainingMode = 0;

%% =========================================================
% 2. MODELL
%% =========================================================

mdl = "pendulum_RL";
agentBlk = mdl + "/RL Agent";

load_system(mdl);

% Parallel training miatt legyen elmentve
save_system(mdl);

%% =========================================================
% 3. MEGLÉVŐ V4 AGENT BETÖLTÉSE
%% =========================================================

load( ...
    "RL_SAC_V4_positionW1_500ep.mat", ...
    "agent", ...
    "obsInfo", ...
    "actInfo", ...
    "Ts");

fprintf("\nV4 500 epizódos agent betöltve.\n");

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
fprintf( ...
    "Szabad GPU memória: %.2f GB\n", ...
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

    fprintf( ...
        "Már fut parallel pool %d workerrel.\n", ...
        pool.NumWorkers);

end

%% =========================================================
% 8. TRAINING OPTIONS
%% =========================================================

additionalEpisodes = 200;

episodeTime = 10;
maxSteps = round(episodeTime / Ts);

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
% 9. INFO
%% =========================================================

fprintf("\n");
fprintf("=========================================\n");
fprintf(" RL V4 - CONTINUE TRAINING\n");
fprintf("=========================================\n");
fprintf("Kiindulás:               500 episode\n");
fprintf("További episode-ok:      %d\n", ...
    additionalEpisodes);
fprintf("Végső összes episode:    700\n");
fprintf("Max steps / episode:     %d\n", ...
    maxSteps);
fprintf("Sample time:             %.3f s\n", ...
    Ts);
fprintf("Parallel workers:        %d\n", ...
    pool.NumWorkers);
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

saveFile = ...
    "RL_SAC_V4_positionW1_700ep.mat";

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
fprintf(" V4 FOLYTATÁS KÉSZ\n");
fprintf("=========================================\n");

fprintf( ...
    "Training idő: %.2f perc\n", ...
    trainingTime_continue / 60);

fprintf( ...
    "Mentve: %s\n", ...
    saveFile);

fprintf("=========================================\n");

%% =========================================================
% RESET FUNCTION
%% =========================================================

function in = resetRobot(in)

    %% Random kezdeti dőlésszög
    theta0 = deg2rad( ...
        -5 + 10*rand);

    q0episode = [
        0;
        theta0;
        0
    ];

    %% Kezdeti sebességek
    qd0episode = [
        0;
        0;
        0
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