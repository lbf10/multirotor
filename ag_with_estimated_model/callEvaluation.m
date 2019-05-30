clc
addpath('../multiControl/')
addpath('../multiControl/utils')
warning('off','all')

if  isempty(gcp('nocreate'))
    poolobj = parpool('ClusterPandora',68);
%     poolobj = parpool('local',2);
end
addAttachedFiles(poolobj,{'evaluateController.m','paramsToMultirotor.m','../multiControl/'})

filelist = dir('Results/Best');
filelist(1:2) = [];

for it=1:length(filelist)
    if ~filelist(it).isdir
        filename = ['Results/Best/',filelist(it).name];
        evaluateController(filename);
    end
end
