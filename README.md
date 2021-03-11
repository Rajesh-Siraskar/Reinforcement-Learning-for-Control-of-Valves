# Reinforcement Learning for Control of Valves
---------------------------------------------------------------------------------------

## Quick Summary:
* This code accompanies the paper titled "Reinforcement Learning for Control of Valves" https://arxiv.org/abs/2012.14668
* The paper explores RL for optimum control of non-linear systems
* Platform: MATLAB's Reinforcement Learning ToolBox (release R2019a) and Simulink

#### Training the RL controller:
* `code_DDPG_Training.m`: Training code that uses DDPG to train an agent in a staged manner. Uses sm_DDPG_Training_Circuit.slx. This file is run iteratively, using Graded Learning to run on the previously stored model and enhancing it's "learning".    
* `sm_DDPG_Training_Circuit.slx`: Simlulink model to train the agent to control a non-linear valve model

#### Experiment with trained controller: 
* ` sm_Experimental_Setup.slx`: Simulink model to compare the DDPG agent controller with PID, and experiment with various noise signals and noise sources
* `code_Experimental_Setup.m`: Load a pre-trained model (RL controller) and run to see effect. Uses sm_Experimental_Setup.slx 

#### Stability analysis of the RL controller: 
* `code_SA_TF_Estimator.m`: Estimate a transfer function for the RL controller to perform stability analysis
* `sm_StabilityStudy.slx`: Simulink model used to estimate the transfer function

---------------------------------------------------------------------------------------

## Introduction:

The paper https://arxiv.org/abs/2012.14668 explores RL for optimum control of non-linear systems. 

We use the DDPG (Deep Deterministic Policy-Gradient) algorithm to control a non-linear valve modelled based on di Capaci and Scali (2018). While the code and paper use valves as a 'plant', the method and code is easily adaptable to any industrial plant.

Challenges associated with Reinforcement Learning (RL) are outlined in the paper. The paper explores "Graded Learning" to assist in efficiently training an RL agent. We decompose the training task into simpler objectives and train the agent in stages. The Graded Learning parameters will be based on your process and plant. 

Note that Graded Learning is the simplest (and application/practise oriented) form of Curriculum Learning (Narvekar et al., 2020). 

The paper and code uses the following elements as the controlled system:

1. An "industrial" process modelled using a first-order plus time-delay (FOPTD) transfer-function of the form
```
        G(s) = k * exp(-L.s) / (1 + T.s)
        where k = 3.8163, T = 156.46 and L is the time-delay parameter and L = 2.5
```
2. A non-linear valve modelled based on (di Capaci and Scali, 2018), characterized by two parameters:
```
		Static friction or stiction: fS = 8.40
		Dynamic friction: fD = 3.5243
```

## Running the code

### 1. Training the agent:

To train the agent, launch the Simulink model `sm_DDPG_Training_Circuit.slx` and then ensure variables are correctly set in the code file `code_DDPG_Training.m` and excute the code.  

Review/set the following global and "Graded Learning" variables:
1. `MODELS_PATH`: Points to your base path for storing the models. Leave it to 'models' and the code will create a folder if it does not exist.
2. `VERSION`: Version suffix for your model, say "V1", or "Grade-1" etc. Ensure you change this so that a new model is created during each stage of the training process. 
3. `VALVE_SIMULATION_MODEL`: Set to the Simulink model 'sm_DDPG_Training_Circuit'. In case you rename it you will have to set the name here.
4. `USE_PRE_TRAINED_MODEL = false`: To train the first model - or to train only a SINGLE model set to 'false'
	To train a pre-trained model, i.e. apply Graded Learning set USE_PRE_TRAINED_MODEL = true;
5. `PRE_TRAINED_MODEL_FILE = 'Grade_I.mat'`: Set to file name of previous stage model. Example shown here is set to Grade_I model, to continue training an agent and create a Grade_II model. 
6. `MAX_EPISODES = 1000`: This is the maximum episodes a training round lasts. Reduce this initally if you want to test it. However training a stable agent requires 1000 of episodes

Next set the  Graded Learning parameters:

Graded Learning: We trained the agent in SIX stages (Grade-I to Grade-VI) by successively increasing the difficulty of the task. The parameters will be based on your process and plant. For this code, we used the following:

1. `TIME_DELAY` = Time-delay parameter (L) of the FOPTD process. Set as 0.1, 0.5, 1.5, 2.0 and 2.5
2. `fS` = Non-linear valve stiction. We use the following stages 1/10th of 8.5, followed by 1/5th, 1/2, 2/3 and finally full 8.4
3. `fD` = Non-linear valve dynamic friction. We used the same fractions as above for fS for fD, finally ending with the actual value of 3.5243
		   
Suggested Graded Learning stages:
```
- GRADE_I:    TIME_DELAY=0.1; fS = 8.4/10; fD = 3.5243/10
- GRADE_II:   TIME_DELAY=0.5; fS = 8.4/5; fD = 3.5243/5
- GRADE_III:  TIME_DELAY=1.5; fS = 8.4/2; fD = 3.5243/2
- GRADE_IV:   TIME_DELAY=1.5; fS = 8.4/1.5; fD = 3.5243/1.5
- GRADE_V:    TIME_DELAY=2.0, fS = 8.4//1.5; fD = 3.5243/1.5
- GRADE_VI:   TIME_DELAY=2.5, fS = 8.4//1.0; fD = 3.5243/1.0
```

### 2. Experimenting with the trained agent:

To experiment with a trained RL controller/agent, launch the Simulink model `sm_Experimental_Setup.slx` and then ensure variables are correctly set in the code file `code_Experimental_Setup.m` and excute the code.

Variables to be set:
1. `MODELS_PATH`: Points to your base path for storing the models. Default 'models/'
2. `VALVE_SIMULATION_MODEL = sm_Experimental_Setup`: Points to Simulink model used for validation against PID and experimenting with different noise sources etc.
3. `PRE_TRAINED_MODEL_FILE = 'Grade_V.mat'`: Pre-trained model (RL controller) to be tested or validated. Example shows a model called `Grade_V.mat`
4. `TIME_DELAY`, `fS` (stiction) and `fD` (dynamic friction): Variables that represent the physical parameters

### 3. Stability Analysis:
Stability Analysis of the RL controller.
Note that the "System Identification Toolbox" must be installed to estimate transfer-functions

Steps:
1. Ensure Estimated TF block is commented through in Simulink model
2. SIMULATE_MODE = true, for loading 'agent'
      - Check that 'agent' is seen in the MATLAB Workspace
3. Run code, this loads 'agent' and runs the loop once
4. *Manually* run the simulink models to load the 'out' variables
      - Check that 'out' is seen in the MATLAB Workspace
5. SIMULATE_MODE = false for TF estimation mode
      - Set NP and NZ as the number of poles and zeros as estimation parameters
6. Run code again to estimate TFs
---------------------------------------------------------------------------------------

## CODE FILES:

### Training code:

1. `sm_DDPG_Training_Circuit.slx`: Simulink: Non-linear valve model, DDPG agent controller
2. `code_DDPG_Training.m`: MATLAB code: Create a DDPG agent and train using Graded Learning   


### Experiment and verify using trained agent:

1. `sm_Experimental_Setup.slx`:	Simulink: Non-linear valve model, DDPG agent controller. Experiment with noise signals and sources
2. `code_Experimental_Setup.m`:	MATLAB code: Load trained models of choice and run to see effects on scope
3. `sm_PID_Tuning.slx`:	Simulink: Tune a PID controller for comparison

### Stability analysis of RL controller:

1. `code_SA_TF_Estimator.m`: MATLAB code: Estimate a Transfer function for the RL controller
2. `code_SA_Utilities.m`: A small  utilities file for plotting etc.
3. `sm_StabilityStudy.slx`:  Simulink: Model to estimate the transfer function for the RL controller
4. `data_SA_TransferFunctions.mat`: Data file: Store the transfer function    
5. `data_TransferFunctions_NP3_NZ1.mat`: Data file: Store the transfer function (example transfer function with 3 poles and 1 zero)

Note that the "System Identification Toolbox" must be installed to estimate transfer-functions

Please cite as:
```
@misc{siraskar2020reinforcement,
      title={Reinforcement Learning for Control of Valves}, 
      author={Rajesh Siraskar},
      year={2020},
      eprint={2012.14668},
      archivePrefix={arXiv},
      primaryClass={cs.LG}
}
```

##### References:

* Narvekar, S., Peng, B., Leonetti, M., Sinapov, J., Taylor, M.E., Stone, P., 2020. *Curriculum learning for reinforcement learning domains: A framework and survey*. arXiv preprint arXiv:2003.04960

* di Capaci, R.B., Scali, C., 2018. *An augmented PID control structure to compensate for valve stiction*. IFAC-PapersOnLine 51, 799–804.