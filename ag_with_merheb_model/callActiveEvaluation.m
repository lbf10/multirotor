clc
addpath('../multiControl/')
addpath('../multiControl/utils')
warning('off','all')

poolobj = parpool('local',8);
addAttachedFiles(poolobj,{'evaluateActiveController.m','paramsToMultirotor.m','../multiControl/'})

filename = '/export/home/leonardof/multirotor/ag_with_merheb_model/Results/2. 15 end time with 0.05 mr and 4 sequential failures/2a Tentativa/Markovian RLQ-R Active Modified_Active NMAC_Active NMAC_05-Mar-2019 01:11:46_light_iterations.mat';
evaluateActiveController(filename)
filename = '/export/home/leonardof/multirotor/ag_with_merheb_model/Results/2. 15 end time with 0.05 mr and 4 sequential failures/2a Tentativa/RLQ-R Active_Active NMAC_Active NMAC_09-Feb-2019 14:31:46_result.mat';
evaluateActiveController(filename)
filename = '/export/home/leonardof/multirotor/ag_with_merheb_model/Results/2. 15 end time with 0.05 mr and 4 sequential failures/2a Tentativa/RLQ-R Active Modified_Active NMAC_Active NMAC_14-Feb-2019 06:58:49_result.mat';
evaluateActiveController(filename)
filename = '/export/home/leonardof/multirotor/ag_with_merheb_model/Results/2. 15 end time with 0.05 mr and 4 sequential failures/2a Tentativa/RLQ-R Active Modified with PIDD_Active NMAC_Active NMAC_19-Feb-2019 09:01:18_result.mat';
evaluateActiveController(filename)
filename = '/export/home/leonardof/multirotor/ag_with_merheb_model/Results/2. 15 end time with 0.05 mr and 4 sequential failures/2a Tentativa/SOSMC Active_Active NMAC_Active NMAC_24-Feb-2019 12:18:43_result.mat';
evaluateActiveController(filename)
filename = '/export/home/leonardof/multirotor/ag_with_merheb_model/Results/2. 15 end time with 0.05 mr and 4 sequential failures/2a Tentativa/SOSMC Active Direct_None_Active NMAC_28-Feb-2019 20:09:49_result.mat';
evaluateActiveController(filename)
filename = '/export/home/leonardof/multirotor/ag_with_merheb_model/Results/2. 15 end time with 0.05 mr and 4 sequential failures/2a Tentativa/SOSMC Active with PIDD_Active NMAC_Active NMAC_13-Mar-2019 10:01:17_result.mat';
evaluateActiveController(filename)
