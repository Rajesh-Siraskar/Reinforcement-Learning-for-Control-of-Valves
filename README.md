# Reinforcement-Learning-for-Control-of-Valves
This project uses DDPG for "optimal" control of non-linear valves. Uses MATLAB and Simulink

TRAINING FILES:
---------------
1. sm_DDPG_Training_Circuit.slx: 	Simulink: Non-linear valve model, DDPG agent controller
2. code_DDPG_Training.m:		MATLAB code: Create a DDPG agent and train using Graded Learning   

EXPERIMENT and VERIFY using trained agent:
------------------------------------------
1. sm_Experimental_Setup.slx:		Simulink: Non-linear valve model, DDPG agent controller. Experiment with noise signals and sources
2. code_Experimental_Setup.m:		MATLAB code: Load trained models of choice and run to see effects on scope
3. sm_PID_Tuning.slx:			Simulink: Tune a PID controller for comparison

STABILITY ANALYSIS:
-------------------
1. code_SA_TF_Estimator.m               MATLAB code: Estimate a Transfer function for the RL controller
2. code_SA_Utilities.m
3. sm_StabilityStudy.slx                Simulink: Model to estimate the transfer function
4. data_SA_TransferFunctions.mat    	Data file: Store the transfer function    
5. data_TransferFunctions_NP3_NZ1.mat   Data file: Store the transfer function (example transfer function with 3 poles and 1 zero)
