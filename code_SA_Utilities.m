% -----------------------------------------------------------------
% Esimated a TFs
% -----------------------------------------------------------------
% Poles and Zeros:  NP=3; NZ=2;
% Data stores:      InputData_C, OutputData_C, OutputData_P
% Transfer Functions:
% tf_Controller, tf_Plant, tf_Plant_TD
% TF Open and Closed Loop
% TF_OpenLoop_TD, TF_ClosedLoop_TD

% save('RLVC2_TransferFunctions', 'NP', 'NZ', 'tf_Controller', 'tf_Plant', 'tf_Plant_TD', 'TF_OpenLoop_TD', 'TF_ClosedLoop_TD')

% 

load('RLVC2_TransferFunctions.mat');
figure(1); step(TF_ClosedLoop_TD); title ('Closed Loop: Step Response');
figure(2); bode(TF_OpenLoop_TD); title ('Open Loop: Bode plot');
figure(3); margin(TF_OpenLoop_TD);
figure(4); pzmap(TF_OpenLoop_TD); title ('Open Loop: Pole-Zero map');
figure(5); nyquist(TF_OpenLoop_TD); title ('Open Loop: Nyquist plot');
figure(6);  margin(TF_ClosedLoop_TD); 