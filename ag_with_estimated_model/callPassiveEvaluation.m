clc
addpath('../multiControl/')
addpath('../multiControl/utils')
warning('off','all')

if  isempty(gcp('nocreate'))
    poolobj = parpool('local',2);
end
addAttachedFiles(poolobj,{'evaluatePassiveController.m','paramsToMultirotor.m','../multiControl/'})

filename = '/home/lbf10/multirotor/ag_with_merheb_model/Results/2. 15 end time with 0.05 mr and 4 sequential failures/2a Tentativa/RLQ-R Passive Modified_Passive NMAC_Passive NMAC_13-Dec-2018 17:15:46_result.mat';
evaluatePassiveController