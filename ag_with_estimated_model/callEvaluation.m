clc
addpath('../multiControl/')
addpath('../multiControl/utils')
warning('off','all')

poolobj = parpool('local',8);
addAttachedFiles(poolobj,{'evaluatePassiveController.m','paramsToMultirotor.m','../multiControl/'})

evaluatePassiveController