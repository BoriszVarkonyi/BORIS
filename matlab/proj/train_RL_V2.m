%% =========================================================
% SAC RL TRAINING V3
%
% Observation:
% [sRef; s; vCmd; v; theta; thetaDot]
%
% Reward:
% position + velocity + balance + angular velocity + torque
%% =========================================================

clearvars;
clc;
close all;

%% Robot paraméterek
run("init_pendulum.m");

%% Modell
mdl = "pendulum_RL";
agentBlk = mdl + "/RL Agent";

load_system(mdl);

%% RL sample time
Ts = 0.02;

episodeTime = 10;
maxSteps = round(episodeTime / Ts);

%% Observation
obsInfo = rlNumericSpec([6 1]);

obsInfo.Name = "robotObservations";

obsInfo.Description = ...
    "sRef, s, vCmd, v, theta, thetaDot";

%% Action
actInfo = rlNumericSpec([1 1]);

actInfo.Name = "tauCommon";

actInfo.LowerLimit = -tauMax;
actInfo.UpperLimit =  tauMax;

%% Environment
env = rlSimulinkEnv( ...
    mdl, ...
    agentBlk, ...
    obsInfo, ...
    actInfo);

env.ResetFcn = @resetRobot;

%% ÚJ SAC agent
agent = rlSACAgent( ...
    obsInfo, ...
    actInfo);

agent.SampleTime = Ts;

agent.AgentOptions.DiscountFactor = 0.995;

%% GPU
gpu = gpuDevice;

fprintf("GPU: %s\n", gpu.Name);
fprintf("GPU memória: %.2f GB\n", ...
    gpu.AvailableMemory / 1e9);

agent.UseGPUForLearning = true;

%% Environment ellenőrzés
validateEnvironment(env);

%% Parallel pool
pool = gcp("nocreate");

if isempty(pool)
    pool = parpool(2);
end

%% Training options
trainOpts = rlTrainingOptions( ...
    MaxEpisodes = 300, ...
    MaxStepsPerEpisode = maxSteps, ...
    ScoreAveragingWindowLength = 20, ...
    StopTrainingCriteria = "None", ...
    Verbose = true, ...
    Plots = "training-progress");

trainOpts.UseParallel = true;

trainOpts.ParallelizationOptions.Mode = "async";

%% Training
fprintf("\n============================\n");
fprintf("RL V3 TRAINING\n");
fprintf("============================\n");
fprintf("Observations: 6\n");
fprintf("Episodes: %d\n", trainOpts.MaxEpisodes);
fprintf("Steps/episode: %d\n", maxSteps);
fprintf("============================\n\n");

tic;

trainingStats = train( ...
    agent, ...
    env, ...
    trainOpts);

trainingTime = toc;

%% Mentés
saveFile = "RL_SAC_V3_position_300ep.mat";

save( ...
    saveFile, ...
    "agent", ...
    "trainingStats", ...
    "obsInfo", ...
    "actInfo", ...
    "Ts", ...
    "trainingTime");

fprintf("\nTraining kész.\n");
fprintf("Idő: %.2f perc\n", trainingTime/60);
fprintf("Mentve: %s\n", saveFile);

%% =========================================================
% RESET FUNCTION
%% =========================================================

function in = resetRobot(in)

    %% Kezdeti dőlésszög: -5 ... +5 fok
    theta0 = deg2rad(-5 + 10*rand);

    q0episode = [
        0;
        theta0;
        0
    ];

    qd0episode = [
        0;
        0;
        0
    ];

    %% Random sebességreferencia
    vTrainCmdEpisode = -0.15 + 0.30*rand;

    %% Simulink változók
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