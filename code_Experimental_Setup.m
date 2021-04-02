%--------------------------------------------------------------------------
% Reinforcement Learning for Valve Control. V.5.4: 11-Mar. 11pm
% Author:       Rajesh Siraskar
% e-mail:       rajeshsiraskar@gmail.com; siraskar@coventry.ac.uk
% University:   Coventry University, UK, MTech Automotive Engineering
%
% Code:         Experiment and validate a trained RL controller. Compare
%               against PID control.
%               This code accompanies the paper titled "Reinforcement Learning for Control of Valves"
%               https://arxiv.org/abs/2012.14668
% -------------------------------------------------------------------------
%
% To experiment with a trained RL controller/agent, launch the Simulink model `sm_Experimental_Setup.slx` and then ensure 
% variables are correctly set in the code file `code_Experimental_Setup.m` and excute the code.

% Variables to be set:
% 1. `MODELS_PATH`: Points to your base path for storing the models. Default 'models/'
% 2. `VALVE_SIMULATION_MODEL = sm_Experimental_Setup`: Points to Simulink model used for validation against PID and experimenting with different noise sources etc.
% 3. `PRE_TRAINED_MODEL_FILE = 'Grade_V.mat'`: Pre-trained model (RL controller) to be tested or validated. Example shows a model called `Grade_V.mat`
% 4. `TIME_DELAY`, `fS` (stiction) and `fD` (dynamic friction): Variables that represent the physical parameters. Set these to the values that you want
% 		test the RL controller against.

% Suggested Graded Learning stages:
% - GRADE_I:    TIME_DELAY=0.1; fS = 8.4/10; fD = 3.5243/10
% - GRADE_II:   TIME_DELAY=0.5; fS = 8.4/5; fD = 3.5243/5
% - GRADE_III:  TIME_DELAY=1.5; fS = 8.4/2; fD = 3.5243/2
% - GRADE_IV:   TIME_DELAY=1.5; fS = 8.4/1.5; fD = 3.5243/1.5
% - GRADE_V:    TIME_DELAY=2.0, fS = 8.4/1.5; fD = 3.5243/1.5
% - GRADE_VI:   TIME_DELAY=2.5, fS = 8.4/1.0; fD = 3.5243/1.0
%--------------------------------------------------------------------------

clear all;

%% Set paths
MODELS_PATH = 'results/';
VALVE_SIMULATION_MODEL = 'sm_Experimental_Setup'; % Simulink experimentation circuit
RL_AGENT = strcat(VALVE_SIMULATION_MODEL, '/RL Sub-System/RL Agent');

%% GRADED LEARNING models
PRE_TRAINED_MODEL_FILE = 'Grade_I.mat';
%PRE_TRAINED_MODEL_FILE = 'Grade_II.mat';
%PRE_TRAINED_MODEL_FILE = 'Grade_III.mat';
%PRE_TRAINED_MODEL_FILE = 'Grade_IV.mat';

% Physical system parameters. Use iteratively. Suceessively increase
%  difficulty of training task and apply Graded Learning to train the agent
TIME_DELAY = 2.5/2;   % Time delay for process controlled by valve
fS = 8.4000/2;        % Valve dynamic friction
fD = 3.5243/2;        % Valve static friction

% Agent stage to be tested
RL_MODEL_FILE = strcat(MODELS_PATH, PRE_TRAINED_MODEL_FILE);

% Time step. Tf/Ts gives Simulink's simulation time
Ts = 1.0;   % Ts: Sample time (secs)
Tf = 200;   % Tf: Simulation length (secs)
ACCEPTABLE_DELTA = 0.05;

% Load experiences from pre-trained agent    
sprintf('- Load model: %s', PRE_TRAINED_MODEL_FILE)
load(RL_MODEL_FILE,'agent');

% ----------------------------------------------------------------
% Validate the learned agent against the model by simulation
% ----------------------------------------------------------------
% Define observation and action space
NUMBER_OBSERVATIONS = 3;

% Observation Vector 
%  (1) U(k)
%  (2) Error signal
%  (3) Error integral

obsInfo = rlNumericSpec([3 1],...
    'LowerLimit',[-inf -inf 0]',...
    'UpperLimit',[ inf  inf inf]');
obsInfo.Name = 'observations';
obsInfo.Description = 'controlled flow, error, integral of error';
numObservations = obsInfo.Dimension(1);

% obsInfo = rlNumericSpec([NUMBER_OBSERVATIONS 1],...
%     'LowerLimit',[0    -inf -inf]',...             % Actual-flow is limited to 0 on the lower-side
%     'UpperLimit',[inf   inf  inf]');
% obsInfo.Name = 'observations';
% obsInfo.Description = '[actual-signal, error, integrated error]';
% numObservations = obsInfo.Dimension(1);

actInfo = rlNumericSpec([1 1]);
actInfo.Name = 'flow';
numActions = numel(actInfo);

% Intialise the environment with the serialised agent and run the test
sprintf ('\n\n ==== RL for control of valves V.5.1 ====================')
sprintf (' ---- Testing model: %s', PRE_TRAINED_MODEL_FILE)
sprintf (' ---- Parameters: Time-Delay: %3.2f, fS: %3.2f, fD: %3.2f', TIME_DELAY, fS, fD)
        
env = rlSimulinkEnv(VALVE_SIMULATION_MODEL, RL_AGENT, obsInfo, actInfo);
simOpts = rlSimulationOptions('MaxSteps', 2000);
experiences = sim(env, agent, simOpts);
    
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