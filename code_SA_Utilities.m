% --------------------------------------------------------------------------
% Reinforcement Learning for Valve Control. V.5.4: 11-Mar. 11pm
% Author:       Rajesh Siraskar
% e-mail:       rajeshsiraskar@gmail.com; siraskar@coventry.ac.uk
% University:   Coventry University, UK, MTech Automotive Engineering
%
% Code:         Visualization plot for Stability Analysis
% -------------------------------------------------------------------------

load('data_SA_TransferFunctions.mat');
figure(1); step(TF_ClosedLoop_TD); title ('Closed Loop: Step Response');
figure(2); bode(TF_OpenLoop_TD); title ('Open Loop: Bode plot');
figure(3); margin(TF_OpenLoop_TD);
figure(4); pzmap(TF_OpenLoop_TD); title ('Open Loop: Pole-Zero map');
figure(5); nyquist(TF_OpenLoop_TD); title ('Open Loop: Nyquist plot');
figure(6);  margin(TF_ClosedLoop_TD); 