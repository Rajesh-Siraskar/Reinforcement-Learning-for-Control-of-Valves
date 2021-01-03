%--------------------------------------------------------------------------
% Reinforcement Learning for Valve Control
% 
% Function for validating control strategies: PID and RL
%
% Author:       Rajesh Siraskar
% Student ID:   8891169
% e-mail:       siraskar@coventry.ac.uk
% University:   Coventry University, UK
%               Course: KPIT02EEC - MTech Automotive Engineering
%               Faculty: Faculty of Engineering, Environment and Computing
%               School: School of Computing, Electronics and Maths
% -------------------------------------------------------------------------
% Graded Learning Stages:
% GRADE_I    TIME_DELAY=0.1; fS = 8.4/10; fD = 3.5243/10 
% GRADE_II   TIME_DELAY=0.5; fS = 8.4/5; fD = 3.5243/5
% GRADE_III  TIME_DELAY=1.5; fS = 8.4/2; fD = 3.5243/2
% GRADE_IV   TIME_DELAY=1.5; fS = 8.4/1.5; fD = 3.5243/1.5
% GRADE_IV   TIME_DELAY=1.5, fS/1.5 and fD/1.5
% GRADE_V    TIME_DELAY=2.0, fS/1.5 and fD/1.5
% GRADE_VI   TIME_DELAY=2.5, fS/1.0 and fD/1.0
%--------------------------------------------------------------------------
% 07-Oct-2020: Test NORMAL Non_Graded_V1 version
%--------------------------------------------------------------------------

clear all;

% Simulink model for environment simulation
VALVE_SIMULATION_MODEL = 'sm_Experimental_Setup';
RL_AGENT = strcat(VALVE_SIMULATION_MODEL, '/RL Sub-System/RL Agent');

% GRADED LEARNING PLOTS
BASE_PATH = 'D:/Rajesh/RLVC3/RLVC3_Matlab/models/TestModels/';
%PRE_TRAINED_MODEL_FILE = 'Non_Graded_OneShot_V4.R2_10000.mat';%
%PRE_TRAINED_MODEL_FILE = 'Grade_IV_ver.1_1000.mat';
%PRE_TRAINED_MODEL_FILE = 'Grade_IV_ver.2_400.mat';
%PRE_TRAINED_MODEL_FILE = 'Grade_IV_ver.3_600.mat';
PRE_TRAINED_MODEL_FILE = 'Grade_IV_ver.3_600.mat';

TIME_DELAY = 2.5;

TRAINING_MODE = false;
ACCEPTABLE_DELTA = 0.05;

% Agent stage to be tested
RL_MODEL_FILE = strcat(BASE_PATH, PRE_TRAINED_MODEL_FILE);

% Time step. Tf/Ts gives Simulink's simulation time
Ts = 1.0;   % Ts: Sample time (secs)
Tf = 200;   % Tf: Simulation length (secs)

% Load experiences from pre-trained agent    
sprintf('- Testing model: %s', PRE_TRAINED_MODEL_FILE)
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
env = rlSimulinkEnv(VALVE_SIMULATION_MODEL, RL_AGENT, obsInfo, actInfo);
simOpts = rlSimulationOptions('MaxSteps', 2000);
experiences = sim(env, agent, simOpts);
    
% ------------------------------------------------------------------------
% Environment Reset function 
% - Reset if the controlled speed drops to zero or negative or > 15 
% ------------------------------------------------------------------------
function in = localResetFcn(in, RL_System)
    % -------------------------------------------------------------
    % GENERIC TRAINING SIGNALs
    % Randomize Reference_Signal between 0 and 100
    % -------------------------------------------------------------
    block_Reference_Signal = strcat (RL_System, '/Reference_Signal');
    Reference_Signal = randi(100) + rand;
    in = setBlockParameter(in, block_Reference_Signal, ...
        'Value', num2str(Reference_Signal));

    % Randomize initial condition of the flow (0 and 100) 
    block_Actual_Flow = strcat (RL_System, '/Plant/Process/FLOW');    
    Actual_Flow = randi(100) + rand;
    in = setBlockParameter(in, block_Actual_Flow, 'Bias', num2str(Actual_Flow));
end

