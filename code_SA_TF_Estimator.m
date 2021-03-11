% --------------------------------------------------------------------------
% Reinforcement Learning for Valve Control. V.5.4: 11-Mar. 11pm
% Author:       Rajesh Siraskar
% e-mail:       rajeshsiraskar@gmail.com; siraskar@coventry.ac.uk
% University:   Coventry University, UK, MTech Automotive Engineering
%
% Code:         Stability Analysis of the RL controller
%               This code accompanies the paper titled "Reinforcement Learning for Control of Valves"
%               https://arxiv.org/abs/2012.14668
% -------------------------------------------------------------------------
% Steps:
% 1. Ensure Estimated TF block is commented through in Simulink model
% 2. SIMULATE_MODE = true, for loading 'agent'
%       - Check that 'agent' is seen in the MATLAB Workspace
% 3. Run code, this loads 'agent' and runs the loop once
% 4. *Manually* run the simulink models to load the 'out' variables
%       - Check that 'out' is seen in the MATLAB Workspace
% 5. SIMULATE_MODE = false for TF estimation mode
%       Set-up NP and NZ as the number of poles and zeros as estimation
%       parameters
% 6. Run code again to estimate TFs
%
% Summary:
% (1) SIMULATE_MODE = true.  Run for creating the "data" for estimation the TF
% (2) SIMULATE_MODE = false. Run for estimating the TF from data created in previous step
%
% Note that the "System Identification Toolbox" must be installed to estimate transfer-functions
% -------------------------------------------------------------------------

warning ('off','all');

%% Set paths and RL agent/controller to be analyzed
MODELS_PATH = 'models/';
VALVE_SIMULATION_MODEL = 'sm_StabilityStudy'; % Stability study Simulink circuit
PRE_TRAINED_MODEL = 'Grade_I.mat';
    
%% Flags to be set. See instructions above. Two rounds need to be run,
%       (1) SIMULATE_MODE = true.  Run for creating the "data" for estimation the TF
%       (2) SIMULATE_MODE = false. Run for estimating the TF from data created in previous step
SIMULATE_MODE = false;
ESTIMATE_MODE = not(SIMULATE_MODE);

% Physical system parameters. Use iteratively. Suceessively increase
%  difficulty of training task and apply Graded Learning to train the agent
TIME_DELAY = 2.5;   % Time delay for process controlled by valve
fS = 8.4000;        % Valve dynamic friction
fD = 3.5243;        % Valve static friction


%% Simulate for estimating the TF
if (SIMULATE_MODE)
    RL_AGENT = strcat(VALVE_SIMULATION_MODEL, '/RL Sub-System/RL Agent');

    % GRADED LEARNING MODELS
    PRE_TRAINED_MODEL_FILE = strcat(MODELS_PATH, PRE_TRAINED_MODEL);

    ACCEPTABLE_DELTA = 0.05;

    % Time step. Tf/Ts gives Simulink's simulation time
    Ts = 1.0;   % Ts: Sample time (secs)
    Tf = 200;   % Tf: Simulation length (secs)

    % Run simulation for collecting data to estimate a Transer Function
    % simout = RunSimulation(VALVE_SIMULATION_MODEL, PRE_TRAINED_MODEL_FILE, RL_AGENT);
    % Load experiences from pre-trained agent    
    sprintf('- Estimating Transfer Function for model: %s', PRE_TRAINED_MODEL_FILE)
    load(PRE_TRAINED_MODEL_FILE,'agent');

    % ----------------------------------------------------------------
    % Validate the learned agent against the model by simulation
    % ----------------------------------------------------------------
    % Define observation and action space
    NUMBER_OBSERVATIONS = 3;
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
    simOpts = rlSimulationOptions('MaxSteps', 500);
    experiences = sim(env, agent, simOpts);
else 
    % Run the Simulink Model to collect data in 'out' variable

    % Esimating a TF
    NP=3; NZ=2;

    % Estimate Controller TF
    InputData_C = out.InputData_C.Data(:);
    OutputData_C = out.OutputData_C.Data(:);
    OutputData_P = out.OutputData_P.Data(:);

    uC = InputData_C;
    yC = OutputData_C;
    signal_length = min(size(yC), size(uC));
    yC = yC(1:signal_length);
    uC = uC(1:signal_length);
    tfdata_C = iddata(yC, uC, Ts);

    tf_Controller = tfest(tfdata_C, NP, NZ);
    
    %% Estimate transfer-function for the entire Plant (process + non-linear valve)
    % In case you want to use the industrial-process TF separately, we
    % could use it directly and estimate ONLY for the non-linear valve too
    % s = tf('s');
    % tf_Plant_TD = 3.8163 / (156.46*s + 1) * exp(-2.5*s);
    % --------------------------------------------------
    InputData_P = OutputData_C;
    uP = InputData_P;
    yP = OutputData_P;
    signal_length = min(size(yP), size(uP));
    yP = yP(1:signal_length);
    uP = uP(1:signal_length);
    tfdata_P = iddata(yP, uP, Ts);

    tf_Plant = tfest(tfdata_P, NP, NZ);
    tf_Plant_TD = tf_Plant;
    
    % ------------------------------------------------- 
    % TF Open and Closed Loop
    TF_OpenLoop_TD = tf_Controller*tf_Plant_TD;
    TF_ClosedLoop_TD = feedback(tf_Plant_TD*tf_Controller, 1);
    
    figure(1); step(TF_ClosedLoop_TD); title ('Closed Loop: Step Response');
    figure(2); bode(TF_OpenLoop_TD); title ('Open Loop: Bode plot');
    figure(3); margin(TF_OpenLoop_TD); 
    figure(4); pzmap(TF_OpenLoop_TD); title ('Open Loop: Pole-Zero map');
    figure(5); nyquist(TF_OpenLoop_TD); title ('Open Loop: Nyquist plot');
    
    % Save TFs
    save('RL_TransferFunctions_V5', 'NP', 'NZ', 'tf_Controller', 'tf_Plant', 'tf_Plant_TD', 'TF_OpenLoop_TD', 'TF_ClosedLoop_TD')
end

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