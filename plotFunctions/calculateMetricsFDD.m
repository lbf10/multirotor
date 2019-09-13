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
        controller(it).graphs.rateSS = [controller(it).graphs.rateSS (mean([controller(it).metrics(auxIndex).simulationSuccess])-reference)];
        % simulationSuccess
        reference = controller(it).metrics(1).RMSPositionError;
        controller(it).graphs.rateEp = [controller(it).graphs.rateEp (mean([controller(it).metrics(auxIndex).RMSPositionError])-reference)];
        % simulationSuccess
        reference = controller(it).metrics(1).RMSPower;
        controller(it).graphs.rateP = [controller(it).graphs.rateP (mean([controller(it).metrics(auxIndex).RMSPower])-reference)];
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
        controller(it).graphs.delaySS = [controller(it).graphs.delaySS (mean([controller(it).metrics(auxIndex).simulationSuccess])-reference)];
        % simulationSuccess
        reference = controller(it).metrics(1).RMSPositionError;
        controller(it).graphs.delayEp = [controller(it).graphs.delayEp (mean([controller(it).metrics(auxIndex).RMSPositionError])-reference)];
        % simulationSuccess
        reference = controller(it).metrics(1).RMSPower;
        controller(it).graphs.delayP = [controller(it).graphs.delayP (mean([controller(it).metrics(auxIndex).RMSPower])-reference)];
    end   
    data = controller(it);
    save([saveDir,'/evaluationFDDMetrics_',data.name{1}],'data');
    disp(['Finished controller ',controller(it).name]);
%     end
end
%% All controllers FDD graphs
plotOrder = [2,8,7,6,5,4,3,1];
% FDD rate X simulationSuccess / rate variation
figure
names = [];
for it=1:length(plotOrder)
    names = [names; controller(plotOrder(it)).name];
    plot(controller(plotOrder(it)).graphs.rates,controller(plotOrder(it)).graphs.rateSS,'*-','LineWidth',2)
    set ( gca, 'xdir', 'reverse' )
    hold on
end
title('System stability variation for FDD hit rate decrease')
xlabel('FDD hit rate')
ylabel('Simulation success variation')
% legend(names)
legend('2.2.1 (PID)','2.2.2.1 (SOSMC)','2.2.2.2 (SOSMC with PIDD)','2.2.2.3 (SOSMC Direct)','2.2.3.1 (R-LQR)','2.2.3.2 (R-LQR with rotor failures)','2.2.3.3 (R-LQR with rotor failures and PIDD)','2.2.4 (Mode-Dependent Markovian)')
savefig([saveDir,'/fddRateXss.fig'])
% FDD rate X RMSPositionError / rate variation
figure
names = []
for it=1:length(plotOrder)
    names = [names; controller(plotOrder(it)).name];
    plot(controller(plotOrder(it)).graphs.rates,controller(plotOrder(it)).graphs.rateEp,'*-','LineWidth',2)
    set ( gca, 'xdir', 'reverse' )
    hold on
end
title('Position error variation for FDD hit rate decrease')
xlabel('FDD hit rate')
ylabel('RMS Position error variation (m)')
% legend(names)
legend('2.2.1 (PID)','2.2.2.1 (SOSMC)','2.2.2.2 (SOSMC with PIDD)','2.2.2.3 (SOSMC Direct)','2.2.3.1 (R-LQR)','2.2.3.2 (R-LQR with rotor failures)','2.2.3.3 (R-LQR with rotor failures and PIDD)','2.2.4 (Mode-Dependent Markovian)')
savefig([saveDir,'/fddRateXep.fig'])
% FDD rate X RMSPositionError / rate variation
figure
names = []
for it=1:length(plotOrder)
    names = [names; controller(plotOrder(it)).name];
    plot(controller(plotOrder(it)).graphs.rates,controller(plotOrder(it)).graphs.rateP,'*-','LineWidth',2)
    set ( gca, 'xdir', 'reverse' )
    hold on
end
title('RMS Power variation for FDD hit rate decrease')
xlabel('FDD hit rate')
ylabel('RMS Power variation (W)')
% legend(names)
legend('2.2.1 (PID)','2.2.2.1 (SOSMC)','2.2.2.2 (SOSMC with PIDD)','2.2.2.3 (SOSMC Direct)','2.2.3.1 (R-LQR)','2.2.3.2 (R-LQR with rotor failures)','2.2.3.3 (R-LQR with rotor failures and PIDD)','2.2.4 (Mode-Dependent Markovian)')
savefig([saveDir,'/fddRateXpow.fig'])

% FDD delay X simulationSuccess / delay variation
figure
names = []
for it=1:length(plotOrder)
    names = [names; controller(plotOrder(it)).name];
    plot(controller(plotOrder(it)).graphs.delays,controller(plotOrder(it)).graphs.delaySS,'*-','LineWidth',2)
    hold on
end
title('System stability variation for FDD delay increase')
xlabel('FDD delay (s)')
ylabel('Simulation success variation')
% legend(names)
legend('2.2.1 (PID)','2.2.2.1 (SOSMC)','2.2.2.2 (SOSMC with PIDD)','2.2.2.3 (SOSMC Direct)','2.2.3.1 (R-LQR)','2.2.3.2 (R-LQR with rotor failures)','2.2.3.3 (R-LQR with rotor failures and PIDD)','2.2.4 (Mode-Dependent Markovian)')
savefig([saveDir,'/fddDelayXss.fig'])
% FDD delay X RMSPositionError / delay variation
figure
names = []
for it=1:length(plotOrder)
    names = [names; controller(plotOrder(it)).name];
    plot(controller(plotOrder(it)).graphs.delays,controller(plotOrder(it)).graphs.delayEp,'*-','LineWidth',2)
    hold on
end
title('Position error variation for FDD delay increase')
xlabel('FDD delay (s)')
ylabel('RMS Position error variation (m)')
% legend(names)
legend('2.2.1 (PID)','2.2.2.1 (SOSMC)','2.2.2.2 (SOSMC with PIDD)','2.2.2.3 (SOSMC Direct)','2.2.3.1 (R-LQR)','2.2.3.2 (R-LQR with rotor failures)','2.2.3.3 (R-LQR with rotor failures and PIDD)','2.2.4 (Mode-Dependent Markovian)')
savefig([saveDir,'/fddDelayXep.fig'])
% FDD delay X RMSPositionError / delay variation
figure
names = []
for it=1:length(plotOrder)
    names = [names; controller(plotOrder(it)).name];
    plot(controller(plotOrder(it)).graphs.delays,controller(plotOrder(it)).graphs.delayP,'*-','LineWidth',2)
    hold on
end
title('RMS Power variation for FDD delay increase')
xlabel('FDD delay (s)')
ylabel('RMS Power variation (W)')
% legend(names)
legend('2.2.1 (PID)','2.2.2.1 (SOSMC)','2.2.2.2 (SOSMC with PIDD)','2.2.2.3 (SOSMC Direct)','2.2.3.1 (R-LQR)','2.2.3.2 (R-LQR with rotor failures)','2.2.3.3 (R-LQR with rotor failures and PIDD)','2.2.4 (Mode-Dependent Markovian)')
savefig([saveDir,'/fddDelayXpow.fig'])
%% Table
table1 = [0 0 0 0 0 0 0; 0 0 0 0 0 0 0];
for it=1:length(plotOrder)
    table1 = [table1; 0 controller(plotOrder(it)).rateVar.ssVariance controller(plotOrder(it)).rateVar.epVariance controller(plotOrder(it)).rateVar.PVariance ...
                    controller(plotOrder(it)).delayVar.ssVariance controller(plotOrder(it)).delayVar.epVariance controller(plotOrder(it)).delayVar.PVariance];
end
latexTable1 = latex(vpa(table1,4));


