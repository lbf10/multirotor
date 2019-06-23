clc
addpath('../multiControl/')
addpath('../multiControl/utils')
warning('off','all')

% if  isempty(gcp('nocreate'))
    poolobj = parpool('ClusterPandora',68);
%     poolobj = parpool('local',2);
    addAttachedFiles(poolobj,{'evaluateMerhebController.m','paramsToMultirotor.m','../multiControl/@multicopter/multicopter.m','../multiControl/@multicopter/model.m','../multiControl/@multicontrol/multicontrol.m'})
    updateAttachedFiles(poolobj);
    % end

filelist = dir('Results/Best');
filelist(1:2) = [];

for it=1:length(filelist)
    if ~filelist(it).isdir
        filename = ['Results/Best/',filelist(it).name];
        evaluateMerhebController(filename);
    end
end
