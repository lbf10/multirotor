clc
addpath('../multiControl/')
addpath('../multiControl/utils')
warning('off','all')

if  isempty(gcp('nocreate'))
    poolobj = parpool('ClusterPandora',68);
%     poolobj = parpool('local',2);
end
addAttachedFiles(poolobj,{'evaluatePassiveController.m','paramsToMultirotor.m','../multiControl/'})

filelist = dir('Results/Best');
filelist(1:2) = [];

for it=1:length(filelist)
    filename = [filelist(it).folder,'/',filelist(it).name];
    evaluateController(filename);
end