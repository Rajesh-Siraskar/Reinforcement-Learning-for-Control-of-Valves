%--------------------------------------------------------------------------
% Reinforcement Learning for Valve Control. V.5.4: 11-Mar. 11pm
% Author:       Rajesh Siraskar
% e-mail:       rajeshsiraskar@gmail.com; siraskar@coventry.ac.uk
% University:   Coventry University, UK, MTech Automotive Engineering
%
% Code:         DDPG Training. Train a RL controller to control a
%               non-linear valve
% -------------------------------------------------------------------------
% 
% This code accompanies the paper titled "Reinforcement Learning for Control of Valves"
% https://arxiv.org/abs/2012.14668
% The paper explores RL for optimum control of non-linear systems. 
% It uses the DDPG (Deep Deterministic Policy-Gradient) algorithm to control a non-linear valve modelled based on 
%   the paper by di Capaci and Scali (2018), "An augmented PID control structure to compensate for valve stiction"
%
% The paper explores "Graded Learning" to efficiently train an RL agent. We decompose the training task into 
%   simpler objectives and train the agent in stages. These parameters will be based on your process and plant. 
% To follow the paper and code, we use the following:
% (1) An "industrial" process modelled using a first-order plus time-delay (FOPTD) transfer-function of the form 
%  		G(s) = k * exp(-L.s) / (1 + T.s)
% 		where k = 3.8163, T = 156.46 and L is the time-delay parameter = 2.5
% (2) A non-linear valve modelled based on (di Capaci and Scali, 2018), characterized by two parameters:
%		Static friction or stiction: fS = 8.4
%		Dynamic friction: fD = 3.5243
% 
% -------------------------------------------------------------------------
% Code description: Training the agent, using Graded Learning
% -------------------------------------------------------------------------
% 
% To train the agent, launch the Simulink model (sm_DDPG_Training_Circuit.slx) and then ensure variables are correctly 
%   set in the code file (code_DDPG_Training.m) and excute the code.  

% Review/set the following *BASIC* and "Graded Learning" variables:
% 1. MODELS_PATH: Points to your base path for storing the models
% 2. VERSION: Version suffix for your model, say "V1", or "Grade-1" etc. Ensure you change this so that a new model is 
%       created during each stage of the training process. 
% 3. VALVE_SIMULATION_MODEL: Set to the Simulink model 'sm_DDPG_Training_Circuit'
% 4. USE_PRE_TRAINED_MODEL = false: To train the first model - or to train only a SINGLE model set to 'false'
%       To train a pre-trained model, i.e. apply Graded Learning set USE_PRE_TRAINED_MODEL = true;
% 5. PRE_TRAINED_MODEL_FILE = 'Grade_I.mat': Set to file name of previous stage model. Example shown here is set to 
%       Grade_I model, to continue training an agent and create a say a Grade_II model. 
% 6. MAX_EPISODES = 1000: This is the maximum episodes a training round lasts. Reduce this initally if you want to test it. 
%       However training a stable agent requires 1000 of episodes

% Next set the  Graded Learning parameters:

% Graded Learning: We trained the agent in SIX stages (Grade-I to Grade-VI) by successively increasing the difficulty of the task. 
% The parameters will be based on your process and plant. For this project, we used the following:
% 1. TIME_DELAY = Time-delay parameter (L) of the FOPTD process. Start with a low value such as 0.1 and increase in steps 
%       untill final value 2.5
% 2. fS = Non-linear valve stiction. Start with a low value such as 1/10th of actual value, followed by 1/5th, 1/2 etc. and 
%       finally full 8.4
% 3. fD = Non-linear valve dynamic friction. Start with a low value as for fS and end with final value 3.5243

% Suggested Graded Learning stages:
% - GRADE_I:    TIME_DELAY=0.1; fS = 8.4/10; fD = 3.5243/10
% - GRADE_II:   TIME_DELAY=0.5; fS = 8.4/5; fD = 3.5243/5
% - GRADE_III:  TIME_DELAY=1.5; fS = 8.4/2; fD = 3.5243/2
% - GRADE_IV:   TIME_DELAY=1.5; fS = 8.4/1.5; fD = 3.5243/1.5
% - GRADE_V:    TIME_DELAY=2.0, fS = 8.4/1.5; fD = 3.5243/1.5
% - GRADE_VI:   TIME_DELAY=2.5, fS = 8.4/1.0; fD = 3.5243/1.0
% -------------------------------------------------------------------------

clear all;
tstart= datetime();

%% ========================================================================
%% Section 1: To try this RL controller training code as is - 
%%            please review/set variables in this section-1
%% ========================================================================


%% Set paths
MODELS_PATH = 'models/';
VALVE_SIMULATION_MODEL = 'sm_DDPG_Training_Circuit'; % Simulink training circuit

%% Set version for model
% For Graded Learning we apply transfer learning. Point to the pre-trained 
%    model to continue learning on. Do not include the path.
VERSION = 'Grade_II';
USE_PRE_TRAINED_MODEL = true;
PRE_TRAINED_MODEL_FILE = 'Grade_I.mat';

%% Set training parameters
SAVE_AGENT_THRESHOLD = 700;     % Save a point-model at this avg. reward 
STOP_TRAINING = 730;            % Stop model training at this avg. reward 
MAX_REWARD = STOP_TRAINING;     % Stop model training at this avg. reward

%% GRADED LEARNING PARAMETERS
% Physical system parameters. Use iteratively. Suceessively increase
%  difficulty of training task and apply Graded Learning to train the agent
TIME_DELAY = 2.5/2;   % Time delay for process controlled by valve
fS = 8.4000/2;        % Valve dynamic friction
fD = 3.5243/2;        % Valve static friction

%% ========================================================================

%% ========================================================================
%% Section-2: Modify below code only for advanced customization
%% ========================================================================

%% MODEL parameters
% Epsiode and time related
MAX_EPISODES = 1000;
Ts = 1.0;   % Ts: Sample time (secs)
Tf = 150;   % Tf: Simulation length (secs)

AVERAGE_WINDOW = 50;        % Average over 50 time-steps 
ACCEPTABLE_DELTA = 0.05;

% DDPG Hyper-paramaters
criticLearningRate = 1e-03;
actorLearningRate  = 1e-04;
GAMMA = 0.90;
BATCH_SIZE = 64;

% Critic network: Neurons for fully-connected Observsation (state) and Action paths 
neuronsOP_FC1 = 50; neuronsOP_FC2 = 25; neuronsAP_FC1 = 25;

date_today = datetime();
RL_AGENT = strcat(VALVE_SIMULATION_MODEL, '/RL Sub-System/RL Agent');
RL_MODEL_FILE = strcat(MODELS_PATH, 'RL_Model_', VERSION, '.mat');

% Observation Vector is composed of
%  (1) U(k)
%  (2) Error signal
%  (3) Error integral

obsInfo = rlNumericSpec([3 1],...
    'LowerLimit',[-inf -inf 0]',...
    'UpperLimit',[ inf  inf inf]');
obsInfo.Name = 'observations';
obsInfo.Description = 'controlled flow, error, integral of error';
numObservations = obsInfo.Dimension(1);

actInfo = rlNumericSpec([1 1]);
actInfo.Name = 'flow';
numActions = numel(actInfo);

env = rlSimulinkEnv(VALVE_SIMULATION_MODEL, RL_AGENT, obsInfo, actInfo);

% Reset function
env.ResetFcn = @(in)localResetFcn(in, VALVE_SIMULATION_MODEL);
% rng(0);

% Create DDPG agent neuronsOP_FC1 = 50; neuronsOP_FC2 = 25; neuronsAP_FC1 = 25;
statePath = [
    imageInputLayer([numObservations 1 1], 'Normalization', 'none', 'Name', 'State')
    fullyConnectedLayer(neuronsOP_FC1, 'Name', 'CriticStateFC1')
    reluLayer('Name', 'CriticRelu1')
    fullyConnectedLayer(neuronsOP_FC2, 'Name', 'CriticStateFC2')];
actionPath = [
    imageInputLayer([numActions 1 1], 'Normalization', 'none', 'Name', 'Action')
    fullyConnectedLayer(neuronsAP_FC1, 'Name', 'CriticActionFC1')];
commonPath = [
    additionLayer(2,'Name', 'add')
    reluLayer('Name','CriticCommonRelu')
    fullyConnectedLayer(1, 'Name', 'CriticOutput')];

% Critic network
criticNetwork = layerGraph();
criticNetwork = addLayers(criticNetwork,statePath);
criticNetwork = addLayers(criticNetwork,actionPath);
criticNetwork = addLayers(criticNetwork,commonPath);
criticNetwork = connectLayers(criticNetwork,'CriticStateFC2','add/in1');
criticNetwork = connectLayers(criticNetwork,'CriticActionFC1','add/in2');
                  
criticOpts = rlRepresentationOptions('LearnRate', criticLearningRate, 'GradientThreshold',1);

critic = rlRepresentation(criticNetwork,obsInfo,actInfo,'Observation',...
    {'State'},'Action',{'Action'},criticOpts);

actorNetwork = [
    imageInputLayer([numObservations 1 1], 'Normalization', 'none', 'Name', 'State')
    fullyConnectedLayer(3, 'Name', 'actorFC')
    tanhLayer('Name', 'actorTanh')
    fullyConnectedLayer(numActions, 'Name', 'Action')
    ];

actorOptions = rlRepresentationOptions('LearnRate',actorLearningRate,'GradientThreshold',1);

actor = rlRepresentation(actorNetwork,obsInfo,actInfo,'Observation',{'State'},'Action',{'Action'},actorOptions);

agentOpts = rlDDPGAgentOptions(...
    'SampleTime', Ts,...
    'TargetSmoothFactor', 1e-3,...
    'DiscountFactor', GAMMA, ...
    'MiniBatchSize', BATCH_SIZE, ...
    'ExperienceBufferLength', 1e6, ...
    'ResetExperienceBufferBeforeTraining', false, ...
    'SaveExperienceBufferWithAgent', false); 

% Exploration Parameters
% ----------------------
% Ornstein Uhlenbeck (OU) action noise:
%   Variance*sqrt(SampleTime) keep between 1% and 10% of your action range
% Variance is halved by these many samples: 
%       halflife = log(0.5)/log(1-VarianceDecayRate)
% Default: 0.45, 1e-5
DDPG_Variance = 1.5;
DDPG_VarianceDecayRate = 1e-5; % Half-life of 1,000 episodes
% agentOpts.NoiseOptions.Variance = DDPG_Variance;
% agentOpts.NoiseOptions.VarianceDecayRate = DDPG_VarianceDecayRate; 

agent = rlDDPGAgent(actor, critic, agentOpts);

maxepisodes = MAX_EPISODES;
maxsteps = ceil(Tf/Ts);

% For parallel computing: 'UseParallel',true, ...
criticOptions.UseDevice = 'gpu';

% To enable save
trainOpts = rlTrainingOptions(...
    'MaxEpisodes', maxepisodes, ...
    'MaxStepsPerEpisode', maxsteps, ...
    'ScoreAveragingWindowLength', AVERAGE_WINDOW, ...
    'Verbose', false, ...
    'Plots','training-progress',...
    'SaveAgentDirectory', MODELS_PATH, ...
    'SaveAgentValue', SAVE_AGENT_THRESHOLD, ...
    'SaveAgentCriteria','AverageReward', ...
    'StopTrainingCriteria','AverageReward',...
    'StopTrainingValue', STOP_TRAINING);

% To plot network architectures
% Show neural network architecture
% actorNet = getModel(getActor(agent));
% criticNet = getModel(getCritic(agent));
% plot(actorNet); title("Actor Network");
% plot(criticNet); title("Critic Network");
% % Print parameters
% actorNet; actorNet.Layers;
% % Print parameters
% criticNet; criticNet.Layers;

if USE_PRE_TRAINED_MODEL    
    % Load experiences from pre-trained agent    
    sprintf('- RLVC: Loading pre-trained model: %s', PRE_TRAINED_MODEL_FILE)
    RL_MODEL_FILE = strcat(MODELS_PATH, PRE_TRAINED_MODEL_FILE);                
    load(RL_MODEL_FILE,'agent');
else
    agent = rlDDPGAgent(actor, critic, agentOpts);
end

% Train the agent or Load a pre-trained model and run in Suimulink
sprintf ('\n\n ==== RL for control of valves V.5.1 ====================')
sprintf ('\n ---- Ver.: %s, Time-Delay: %3.2f, fS: %3.2f, fD: %3.2f\n', VERSION, TIME_DELAY, fS, fD)
if (USE_PRE_TRAINED_MODEL)
    sprintf (' ---- Graded Learning in progress. Using pre-trained model: %s', PRE_TRAINED_MODEL_FILE)
end
% Train the agent
trainingStats = train(agent, env, trainOpts);
% Save agent
nEpisodes = length(trainingStats.EpisodeIndex);
fname = strcat(MODELS_PATH, VERSION, '.mat');
sprintf ('Saving file: %s', fname)
save(fname, "agent");
display("End training: ") 
tend = datetime();

display("Elapsed time:")
telapsed = tend - tstart;
telapsed

% ------------------------------------------------------------------------
% Environment Reset function 
% Randomize Reference_Signal between 0 and 100
% Reset if the controlled speed drops below zero or exceeds 100 
% ------------------------------------------------------------------------
function in = localResetFcn(in, RL_System)
    block_Reference_Signal = strcat (RL_System, '/Reference_Signal');
    Reference_Signal = 20+randi(80) + rand;
    in = setBlockParameter(in, block_Reference_Signal, ...
        'Value', num2str(Reference_Signal));

    % Randomize initial condition of the flow (0 and 100) 
    block_Actual_Flow = strcat (RL_System, '/Plant/Process/FLOW');    
    Actual_Flow = 20 + randi(80) + rand;
    in = setBlockParameter(in, block_Actual_Flow, 'Bias', num2str(Actual_Flow));
end
