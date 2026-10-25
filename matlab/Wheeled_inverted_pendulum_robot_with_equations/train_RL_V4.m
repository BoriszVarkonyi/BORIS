%% =========================================================
% RL V4 - SAC TRAINING
%
% Observation:
%   [sRef; s; vCmd; v; theta; thetaDot]
%
% Action:
%   tauCommon
%
% V4 változás:
%   position reward weight:
%       0.5 -> 1.0
%
% Training:
%   500 episode
%% =========================================================

clearvars;
clc;
close all;

%% =========================================================
% 1. ROBOT PARAMÉTEREK
%% =========================================================

run("init_pendulum.m");

% Training alatt a random vTrainCmd ágat használjuk
trainingMode = 0;

%% =========================================================
% 2. MODELL
%% =========================================================

mdl = "pendulum_RL";
agentBlk = mdl + "/RL Agent";

load_system(mdl);

% Parallel traininghez elmentett modell kell
save_system(mdl);

%% =========================================================
% 3. RL MINTAVÉTELI IDŐ
%% =========================================================

Ts = 0.02;

episodeTime = 10;

maxSteps = round(episodeTime / Ts);

%% =========================================================
% 4. OBSERVATION SPACE
%
% [sRef
%  s
%  vCmd
%  v
%  theta
%  thetaDot]
%% =========================================================

obsInfo = rlNumericSpec([6 1]);

obsInfo.Name = "robotObservations";

obsInfo.Description = ...
    "sRef, s, vCmd, v, theta, thetaDot";

%% =========================================================
% 5. ACTION SPACE
%% =========================================================

actInfo = rlNumericSpec([1 1]);

actInfo.Name = "tauCommon";

actInfo.LowerLimit = -tauMax;
actInfo.UpperLimit =  tauMax;

%% =========================================================
% 6. RL ENVIRONMENT
%% =========================================================

env = rlSimulinkEnv( ...
    mdl, ...
    agentBlk, ...
    obsInfo, ...
    actInfo);

env.ResetFcn = @resetRobot;

%% =========================================================
% 7. ÚJ SAC AGENT
%% =========================================================

agent = rlSACAgent( ...
    obsInfo, ...
    actInfo);

agent.SampleTime = Ts;

agent.AgentOptions.DiscountFactor = 0.995;

%% =========================================================
% 8. GPU
%% =========================================================

gpu = gpuDevice;

fprintf("GPU: %s\n", gpu.Name);

fprintf( ...
    "GPU memória: %.2f GB\n", ...
    gpu.AvailableMemory / 1e9);

agent.UseGPUForLearning = true;

%% =========================================================
% 9. ENVIRONMENT ELLENŐRZÉS
%% =========================================================

validateEnvironment(env);

fprintf("RL environment validálva.\n");

%% =========================================================
% 10. PARALLEL POOL
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
% 11. TRAINING OPTIONS
%% =========================================================

numberOfEpisodes = 500;

trainOpts = rlTrainingOptions( ...
    MaxEpisodes = numberOfEpisodes, ...
    MaxStepsPerEpisode = maxSteps, ...
    ScoreAveragingWindowLength = 20, ...
    StopTrainingCriteria = "None", ...
    Verbose = true, ...
    Plots = "training-progress");

trainOpts.UseParallel = true;

trainOpts.ParallelizationOptions.Mode = "async";

%% =========================================================
% 12. TRAINING INFO
%% =========================================================

fprintf("\n");
fprintf("=========================================\n");
fprintf(" RL V4 TRAINING\n");
fprintf("=========================================\n");
fprintf("Observations:              6\n");
fprintf("Episodes:                  %d\n", ...
    numberOfEpisodes);
fprintf("Steps / episode:           %d\n", ...
    maxSteps);
fprintf("Episode time:              %.1f s\n", ...
    episodeTime);
fprintf("Sample time:               %.3f s\n", ...
    Ts);
fprintf("Position cost weight:      1.0\n");
fprintf("Parallel workers:          %d\n", ...
    pool.NumWorkers);
fprintf("=========================================\n\n");

%% =========================================================
% 13. TRAINING
%% =========================================================

tic;

trainingStats = train( ...
    agent, ...
    env, ...
    trainOpts);

trainingTime = toc;

%% =========================================================
% 14. MENTÉS
%% =========================================================

saveFile = "RL_SAC_V4_positionW1_500ep.mat";

save( ...
    saveFile, ...
    "agent", ...
    "trainingStats", ...
    "obsInfo", ...
    "actInfo", ...
    "Ts", ...
    "trainingTime");

%% =========================================================
% 15. EREDMÉNY
%% =========================================================

fprintf("\n");
fprintf("=========================================\n");
fprintf(" RL V4 TRAINING KÉSZ\n");
fprintf("=========================================\n");

fprintf( ...
    "Training idő: %.2f perc\n", ...
    trainingTime / 60);

fprintf( ...
    "Mentve: %s\n", ...
    saveFile);

fprintf("=========================================\n");

%% =========================================================
% RESET FUNCTION
%% =========================================================

function in = resetRobot(in)

    %% Véletlen kezdeti dőlésszög
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

    % FONTOS:
    % trainingMode = 0 -> vTrainCmd ág
    in = setVariable( ...
        in, ...
        "trainingMode", ...
        0);

end