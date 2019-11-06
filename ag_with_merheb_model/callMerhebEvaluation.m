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

    filename = 'Results/2. 15 end time with 0.05 mr and 4 sequential failures/Markoviano_com_P/Markovian RLQ-R Passive Modified_Passive NMAC_Passive NMAC_01-Nov-2019 00:02:47_iterations.mat';
    evaluateMerhebController(filename);
    
% filelist = dir('Results/Best');
% filelist(1:2) = [];
% 
% for it=1:length(filelist)
%     if ~filelist(it).isdir
%         filename = ['Results/Best/',filelist(it).name];
%         evaluateMerhebController(filename);
%     end
% end
