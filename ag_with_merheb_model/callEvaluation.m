clc
addpath('../multiControl/')
addpath('../multiControl/utils')
warning('off','all')

poolobj = parpool('local',2);
addAttachedFiles(poolobj,{'evaluatePassiveController.m','paramsToMultirotor.m','../multiControl/'})

filename = '/home/lbf10/multirotor/ag_with_merheb_model/Results/2. 15 end time with 0.05 mr and 4 sequential failures/2a Tentativa/Adaptive_Passive NMAC_Passive NMAC_14-Jan-2019 22:19:51_iterations.mat';
evaluatePassiveController
filename = '/home/lbf10/multirotor/ag_with_merheb_model/Results/2. 15 end time with 0.05 mr and 4 sequential failures/2a Tentativa/Adaptive Direct_None_Passive NMAC_29-Jan-2019 23:09:42_result.mat';
evaluatePassiveController
filename = '/home/lbf10/multirotor/ag_with_merheb_model/Results/2. 15 end time with 0.05 mr and 4 sequential failures/2a Tentativa/Adaptive with PIDD_Passive NMAC_Passive NMAC_22-Jan-2019 18:37:58_iterations.mat';
evaluatePassiveController
filename = '/home/lbf10/multirotor/ag_with_merheb_model/Results/2. 15 end time with 0.05 mr and 4 sequential failures/2a Tentativa/Markovian RLQ-R Passive Modified_Passive NMAC_Passive NMAC_30-Jan-2019 16:05:28_light_iterations.mat';
evaluatePassiveController
filename = '/home/lbf10/multirotor/ag_with_merheb_model/Results/2. 15 end time with 0.05 mr and 4 sequential failures/2a Tentativa/RLQ-R Passive Modified with PIDD_Passive NMAC_Passive NMAC_16-Dec-2018 05:55:25_iterations.mat';
evaluatePassiveController
filename = '/home/lbf10/multirotor/ag_with_merheb_model/Results/2. 15 end time with 0.05 mr and 4 sequential failures/2a Tentativa/SOSMC Passive_Passive NMAC_Passive NMAC_02-Jan-2019 09:41:43_result.mat';
evaluatePassiveController
filename = '/home/lbf10/multirotor/ag_with_merheb_model/Results/2. 15 end time with 0.05 mr and 4 sequential failures/2a Tentativa/SOSMC Passive Direct_None_Passive NMAC_08-Jan-2019 15:19:25_iterations.mat';
evaluatePassiveController
filename = '/home/lbf10/multirotor/ag_with_merheb_model/Results/2. 15 end time with 0.05 mr and 4 sequential failures/2a Tentativa/SOSMC Passive with PIDD_Passive NMAC_Passive NMAC_06-Jan-2019 05:32:13_result.mat';
evaluatePassiveController
filename = '/home/lbf10/multirotor/ag_with_merheb_model/Results/2. 15 end time with 0.05 mr and 4 sequential failures/2a Tentativa/PID_Passive NMAC_Passive NMAC_09-Dec-2018 08:08:46_result.mat';
evaluatePassiveController
filename = '/home/lbf10/multirotor/ag_with_merheb_model/Results/2. 15 end time with 0.05 mr and 4 sequential failures/1a Tentativa (Cluster parou)/PID_Passive NMAC_Passive NMAC_05-Dec-2018 14:10:50_iterations.mat';
evaluatePassiveController
