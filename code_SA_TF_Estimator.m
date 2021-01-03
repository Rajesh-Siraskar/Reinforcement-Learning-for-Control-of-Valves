%------------------------------------------------------------------------------------------------------------
% Reinforcement Learning for Valve Control
% Stability Analysis
% V.2.0 09-Feb-2020: Data file: 09-Feb-2020_AllData.mat
% V.3.0 09-Feb-2020: Use the original transfer function for the plant 3.8163 / (156.46s + 1) * exp(-2.5s)
% V 4.0 22-Feb-202: Correct for non-linear valve model
% ----------------------------------------------------------------------------------------------------------
warning ('off','all');

% STEP:
% 1. Ensure Estimated TF block is commented through in Simulink model
% 2. SIMULATE_MODE = true, for loading 'agent'
%       - Check that 'agent' is seen in the MATLAB Workspace
% 3. Run code, this loads 'agent' and runs the loop once
% 4. *Manually* run the simulink models to load the 'out' variables
%       - Check that 'out' is seen in the MATLAB Workspace
% 5. SIMULATE_MODE = false for TF estimation mode
% 6. Run code again to estimate TFs

SIMULATE_MODE = false;
ESTIMATE_MODE = not(SIMULATE_MODE);

if (SIMULATE_MODE)
    TRAINING_MODE = true; % Use STEP signal reference
    
    clear all;
    % Simulink model for environment simulation
    VALVE_SIMULATION_MODEL = 'RLVC2_StabilityStudy';
    RL_AGENT = strcat(VALVE_SIMULATION_MODEL, '/RL Sub-System/RL Agent');

    % VERSION RLVC2 GRADED LEARNING MODELS
    BASE_PATH = 'D:/RajeshS/RLVC2/models/ValidationModels/';
    PRE_TRAINED_MODEL_FILE = strcat(BASE_PATH, 'Grade_VI.Ver2_2000.mat');

    % Basic constants
    TIME_DELAY = 2.5;
    ACCEPTABLE_DELTA = 0.05;

    % Time step. Tf/Ts gives Simulink's simulation time
    Ts = 1.0;   % Ts: Sample time (secs)
    Tf = 200;   % Tf: Simulation length (secs)

    % Run simulation for collecting data to estimate a Transer Function
    % simout = RunSimulation(VALVE_SIMULATION_MODEL, PRE_TRAINED_MODEL_FILE, RL_AGENT);
    % Load experiences from pre-trained agent    
    % Load experiences from pre-trained agent    
    sprintf('- Estimating TF for model: %s', PRE_TRAINED_MODEL_FILE)
    load(PRE_TRAINED_MODEL_FILE,'agent');

    % ----------------------------------------------------------------
    % Validate the learned agent against the model by simulation
    % ----------------------------------------------------------------
    % Define observation and action space
    NUMBER_OBSERVATIONS = 3;

    % Ver. 13: Observation Vector:  
    %  (1) Integral of |(U(k) - U(k-1)|
    %  (2) Error signal
    %  (3) Error integral

    obsInfo = rlNumericSpec([NUMBER_OBSERVATIONS 1],...
        'LowerLimit',[0    -inf -inf]',...             % Actual-flow is limited to 0 on the lower-side
        'UpperLimit',[inf   inf  inf]');
    obsInfo.Name = 'observations';
    obsInfo.Description = '[actual-signal, error, integrated error]';
    numObservations = obsInfo.Dimension(1);

    actInfo = rlNumericSpec([1 1]);
    actInfo.Name = 'flow';
    numActions = numel(actInfo);

    % Intialise the environment with the serialised agent and run the test
    env = rlSimulinkEnv(VALVE_SIMULATION_MODEL, RL_AGENT, obsInfo, actInfo);
    simOpts = rlSimulationOptions('MaxSteps', 500);
    experiences = sim(env, agent, simOpts);
else 
    % Run the Simulink Model to collect data in 'out' variable

    % Esimating a TF
    % NP=3; NZ=2;
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
    
    % Plant TF
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
    save('RLVC2_TransferFunctions_NP3_NZ1', 'NP', 'NZ', 'tf_Controller', 'tf_Plant', 'tf_Plant_TD', 'TF_OpenLoop_TD', 'TF_ClosedLoop_TD')
end

% ------------------------------------------------------------------------
% Environment Reset function 
% - Reset if the controlled speed drops to zero or negative or > 15 
% ------------------------------------------------------------------------
function in = localResetFcn(in, RL_System)
    % -------------------------------------------------------------
    % GENERIC TRAINING SIGNALs
    % -------------------------------------------------------------
    % Randomize Reference_Signal between 2 and 12
    % randi(11) = 1 to 11 random integer
    % rand = 0-1 real number
    block_Reference_Signal = strcat (RL_System, '/Reference_Signal');
    Reference_Signal = 1.0 + randi(10) + rand;
    in = setBlockParameter(in, block_Reference_Signal, ...
        'Value', num2str(Reference_Signal));

    % Randomize initial condition of the flow (2 to 12) 
    block_Actual_Flow = strcat (RL_System, '/Valve Model/FLOW');    
    Actual_Flow = 1.0 + randi(10) + rand;;
    in = setBlockParameter(in, block_Actual_Flow, 'Bias', num2str(Actual_Flow));    
end

