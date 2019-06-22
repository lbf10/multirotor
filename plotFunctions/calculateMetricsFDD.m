% clear all

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
%     try
    aux = strsplit(subFolders(it).name,'Evaluation_');
    % Get controller name from folder name
    controller(it).name = aux(2);
    % Load evaluation data
    data = load([subFolders(it).folder,'/',subFolders(it).name,'/evaluationFDDResult.mat'],'samples');  
    controller(it).data = data.samples;
    %% Overall sucess rate
    controller(it).metrics = [controller(it).data{:,9}];
    numberOfSimulations = length(controller(it).metrics);
    numberOfSuccesses = sum([controller(it).metrics.simulationSuccess] == 1);
    numberOfFailures = numberOfSimulations-numberOfSuccesses;
    controller(it).perfectSuccessRate = numberOfSuccesses/numberOfSimulations;
    controller(it).meanSuccessRate = mean([controller(it).metrics.simulationSuccess]);
    %% Robustness metrics  
    controller(it).fddRate = [controller(it).data{:,10}];
    controller(it).fddDelay = [controller(it).data{:,11}];
    indexRateVariation = find(controller(it).fddDelay == 0);
    indexDelayVariation = find(controller(it).fddRate == 1);
    % Variances for rate variation
    controller(it).rateVar.ssVariance = var([controller(it).metrics(indexRateVariation).simulationSuccess]);
    controller(it).rateVar.epVariance = var([controller(it).metrics(indexRateVariation).RMSPositionError]);
    controller(it).rateVar.PVariance = var([controller(it).metrics(indexRateVariation).RMSPower]);
    % Variances for delay variation
    controller(it).delayVar.ssVariance = var([controller(it).metrics(indexDelayVariation).simulationSuccess]);
    controller(it).delayVar.epVariance = var([controller(it).metrics(indexDelayVariation).RMSPositionError]);
    controller(it).delayVar.PVariance = var([controller(it).metrics(indexDelayVariation).RMSPower]);
    %% FDD rate graphs
    controller(it).graphs.rates = sort(unique(controller(it).fddRate),'descend');
    controller(it).graphs.rateSS = [];
    controller(it).graphs.rateEp = [];
    controller(it).graphs.rateP = [];
    for jt=1:length(controller(it).graphs.rates)
        auxIndex = find(controller(it).fddRate == controller(it).graphs.rates(jt));
        auxIndex = auxIndex(ismember(auxIndex,indexRateVariation));
        % simulationSuccess
        reference = controller(it).metrics(1).simulationSuccess;
        controller(it).graphs.rateSS = [controller(it).graphs.rateSS 100*(mean([controller(it).metrics(auxIndex).simulationSuccess])-reference)/reference];
        % simulationSuccess
        reference = controller(it).metrics(1).RMSPositionError;
        controller(it).graphs.rateEp = [controller(it).graphs.rateEp 100*(mean([controller(it).metrics(auxIndex).RMSPositionError])-reference)/reference];
        % simulationSuccess
        reference = controller(it).metrics(1).RMSPower;
        controller(it).graphs.rateP = [controller(it).graphs.rateP 100*(mean([controller(it).metrics(auxIndex).RMSPower])-reference)/reference];
    end
    %% FDD delay graphs
    controller(it).graphs.delays = sort(unique(controller(it).fddDelay),'ascend');
    controller(it).graphs.delaySS = [];
    controller(it).graphs.delayEp = [];
    controller(it).graphs.delayP = [];
    for jt=1:length(controller(it).graphs.delays)
        auxIndex = find(controller(it).fddDelay == controller(it).graphs.delays(jt));
        auxIndex = auxIndex(ismember(auxIndex,indexDelayVariation));
        % simulationSuccess
        reference = controller(it).metrics(1).simulationSuccess;
        controller(it).graphs.delaySS = [controller(it).graphs.delaySS 100*(mean([controller(it).metrics(auxIndex).simulationSuccess])-reference)/reference];
        % simulationSuccess
        reference = controller(it).metrics(1).RMSPositionError;
        controller(it).graphs.delayEp = [controller(it).graphs.delayEp 100*(mean([controller(it).metrics(auxIndex).RMSPositionError])-reference)/reference];
        % simulationSuccess
        reference = controller(it).metrics(1).RMSPower;
        controller(it).graphs.delayP = [controller(it).graphs.delayP 100*(mean([controller(it).metrics(auxIndex).RMSPower])-reference)/reference];
    end   
    data = controller(it);
    save([saveDir,'/evaluationFDDMetrics_',data.name{1}],'data');
    disp(['Finished controller ',controller(it).name]);
%     end
end
%% All controllers FDD graphs
% FDD rate X simulationSuccess / rate variation
figure
names = [];
for it=1:length(controller)
    names = [names; controller(it).name];
    plot(controller(it).graphs.rates,controller(it).graphs.rateSS,'*-','LineWidth',2)
    set ( gca, 'xdir', 'reverse' )
    hold on
end
title('System stability variation for FDD hit rate decrease')
xlabel('FDD hit rate')
ylabel('Simulation success variation (%)')
legend(names)
% FDD rate X RMSPositionError / rate variation
figure
names = []
for it=1:length(controller)
    names = [names; controller(it).name];
    plot(controller(it).graphs.rates,controller(it).graphs.rateEp,'*-','LineWidth',2)
    set ( gca, 'xdir', 'reverse' )
    hold on
end
title('Position error variation for FDD hit rate decrease')
xlabel('FDD hit rate')
ylabel('RMS Position error variation (%)')
legend(names)
% FDD rate X RMSPositionError / rate variation
figure
names = []
for it=1:length(controller)
    names = [names; controller(it).name];
    plot(controller(it).graphs.rates,controller(it).graphs.rateP,'*-','LineWidth',2)
    set ( gca, 'xdir', 'reverse' )
    hold on
end
title('RMS Power variation for FDD hit rate decrease')
xlabel('FDD hit rate')
ylabel('RMS Power variation (%)')
legend(names)

% FDD rate X simulationSuccess / delay variation
figure
names = []
for it=1:length(controller)
    names = [names; controller(it).name];
    plot(controller(it).graphs.delays,controller(it).graphs.delaySS,'*-','LineWidth',2)
    hold on
end
title('System stability variation for FDD delay increase')
xlabel('FDD delay (s)')
ylabel('Simulation success variation (%)')
legend(names)
% FDD rate X RMSPositionError / delay variation
figure
names = []
for it=1:length(controller)
    names = [names; controller(it).name];
    plot(controller(it).graphs.delays,controller(it).graphs.delayEp,'*-','LineWidth',2)
    hold on
end
title('Position error variation for FDD delay increase')
xlabel('FDD delay (s)')
ylabel('RMS Position error variation (%)')
legend(names)
% FDD rate X RMSPositionError / delay variation
figure
names = []
for it=1:length(controller)
    names = [names; controller(it).name];
    plot(controller(it).graphs.delays,controller(it).graphs.delayP,'*-','LineWidth',2)
    hold on
end
title('RMS Power variation for FDD delay increase')
xlabel('FDD delay (s)')
ylabel('RMS Power variation (%)')
legend(names)
%% Table
table1 = [0 0 0 0 0 0 0; 0 0 0 0 0 0 0];
for it=1:length(controller)
    table1 = [table1; 0 controller(it).rateVar.ssVariance controller(it).rateVar.epVariance controller(it).rateVar.PVariance ...
                    controller(it).delayVar.ssVariance controller(it).delayVar.epVariance controller(it).delayVar.PVariance];
end
latexTable1 = latex(vpa(table1,2));


