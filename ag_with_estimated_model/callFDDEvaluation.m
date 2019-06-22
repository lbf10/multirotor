clc
addpath('../multiControl/')
addpath('../multiControl/utils')
warning('off','all')

if  isempty(gcp('nocreate'))
%     poolobj = parpool('ClusterPandora',68);
    poolobj = parpool('local',2);
    addAttachedFiles(poolobj,{'evaluateFDD.m','paramsToMultirotor.m','../multiControl/@multicopter/multicopter.m','../multiControl/@multicopter/model.m','../multiControl/@multicontrol/multicontrol.m'})
end

% Get a list of all files and folders in this folder.
files = dir(uigetdir);
% Get a logical vector that tells which is a directory.
dirFlags = [files.isdir];
% Extract only those that are directories.
subFolders = files(dirFlags);
subFolders(1:2) = [];

controller = struct([]);

saveDir = uigetdir('','Choose a folder to save files to');

for it=1:length(subFolders)
    if ~isempty(strfind(subFolders(it).name, 'Active'))
        aux = strsplit(subFolders(it).name,'Evaluation_');
        % Get controller name from folder name
        controller(it).name = aux(2);
        % Load evaluation data
        data = load([subFolders(it).folder,'/',subFolders(it).name,'/evaluationResult.mat'],'samples');  
        controller(it).data = data.samples;
        %% Overall sucess rate
        samplesFields = fields(controller(it).data);
        allSims = [];
        for jt = 1:numel(samplesFields)
            if ~strcmp(samplesFields{jt},'columnNames')
                allSims = [allSims; controller(it).data.(samplesFields{jt})];
            end
        end
        [minValue,minIndex] = min([allSims{:,8}])
        filename = ['Results/Best/',controller(it).name{1},'.mat']
        clear controller
        evaluateFDD(allSims(minIndex,:),filename,saveDir);
    end
end