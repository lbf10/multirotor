% clear all

% Get a list of all files and folders in this folder.
files = dir(uigetdir);
% Get a logical vector that tells which is a directory.
dirFlags = [files.isdir];
% Extract only those that are directories.
subFolders = files(dirFlags);
subFolders(1:2) = [];

controller = struct([]);

for it=1:length(subFolders)
    try
    aux = strsplit(subFolders(it).name,'Evaluation_');
    % Get controller name from folder name
    controller(it).name = aux(2);
    % Load evaluation data
    data = load([subFolders(it).folder,'/',subFolders(it).name,'/evaluationResult.mat'],'samples');  
    controller(it).data = data.samples;
    %% Overall sucess rate
    samplesFields = fields(controller(it).data);
    allMetrics = [];
    for jt = 1:numel(samplesFields)
        if ~strcmp(samplesFields{jt},'columnNames')
            allMetrics = [allMetrics controller(it).data.(samplesFields{jt}){:,9}];
        end
    end
    controller(it).metrics = allMetrics;
    numberOfSimulations = length(controller(it).metrics);
    numberOfSuccesses = sum([controller(it).metrics.simulationSuccess] == 1);
    numberOfFailures = numberOfSimulations-numberOfSuccesses;
    controller(it).perfectSuccessRate = numberOfSuccesses/numberOfSimulations;
    controller(it).meanSuccessRate = mean([controller(it).metrics.simulationSuccess]);
    %% Performance metrics
    indexSuccesful = find([controller(it).metrics.simulationSuccess] == 1);
    rmsPositionError = [controller(it).metrics.RMSPositionError]';
    meanPositionError = [controller(it).metrics.meanPositionError]';
    localVariancePositionError = rmsPositionError.^2-meanPositionError.^2;
    rmsAngularError = [controller(it).metrics.RMSAngularError]';
    meanAngularError = [controller(it).metrics.meanAngularError]';
    localVarianceAngularError = rmsAngularError.^2-meanAngularError.^2;
    rmsPower = [controller(it).metrics.RMSPower]';
    try
    meanPower = [controller(it).metrics.meanPower]';
    localVariancePower = rmsPower.^2-meanPower.^2;
    end
    computationalCost = [controller(it).metrics.meanTime]';
    % Metrics for p_sim == 1
    controller(it).bestPerformance.meanEp = mean(meanPositionError(indexSuccesful));
    controller(it).bestPerformance.meanVarEp = mean(localVariancePositionError(indexSuccesful));
    controller(it).bestPerformance.meanEatt = mean(meanAngularError(indexSuccesful));
    controller(it).bestPerformance.meanVarEatt = mean(localVarianceAngularError(indexSuccesful));
    try
    controller(it).bestPerformance.meanP = mean(meanPower(indexSuccesful));
    controller(it).bestPerformance.meanVarP = mean(localVariancePower(indexSuccesful));
    end
    controller(it).bestPerformance.computationalCost = mean(computationalCost(indexSuccesful));
    % Metrics for all simulations
    controller(it).meanPerformance.meanEp = mean(meanPositionError);
    controller(it).meanPerformance.meanVarEp = mean(localVariancePositionError);
    controller(it).meanPerformance.meanEatt = mean(meanAngularError);
    controller(it).meanPerformance.meanVarEatt = mean(localVarianceAngularError);
    try
    controller(it).meanPerformance.meanP = mean(meanPower);
    controller(it).meanPerformance.meanVarP = mean(localVariancePower);
    end
    controller(it).meanPerformance.computationalCost = mean(computationalCost);
    %% Robustness metrics
    % Aggregate results a0,a1,a2,a3,a4,a5,a6,a7,a1234567.
    
    for jt = 1:numel(samplesFields)
        if ~strcmp(samplesFields{jt},'columnNames')
            metrics = [controller(it).data.(samplesFields{jt}){:,9}];
            controller(it).epRobustness.(samplesFields{jt}) = [];
            controller(it).PRobustness.(samplesFields{jt}) = [];
            controller(it).ssRobustness.(samplesFields{jt}) = [];
            for kt=1:length(metrics)
                if isinf(metrics(kt).RMSPositionError) || metrics(kt).RMSPositionError>1000
                    aux = 5/metrics(kt).simulationSuccess;
                else
                    %aux = metrics(kt).RMSPositionError/metrics(kt).simulationSuccess;
                    aux = metrics(kt).RMSPositionError;
                end
                controller(it).epRobustness.(samplesFields{jt}) = [controller(it).epRobustness.(samplesFields{jt}); aux];
                if isinf(metrics(kt).RMSPower)
                    aux = 400/metrics(kt).simulationSuccess;
                else
                    %aux = metrics(kt).RMSPower/metrics(kt).simulationSuccess;
                    aux = metrics(kt).RMSPower;
                end
                controller(it).PRobustness.(samplesFields{jt}) = [controller(it).PRobustness.(samplesFields{jt}); aux];
                controller(it).ssRobustness.(samplesFields{jt}) = [controller(it).ssRobustness.(samplesFields{jt}); metrics(kt).simulationSuccess];
            end
        end
    end
    % RMS Position Error
    % Sf index
    squaredMeanSf = mean(controller(it).epRobustness.N0.*controller(it).epRobustness.N1234567);
    variance = sum(controller(it).epRobustness.N1234567.^2.-squaredMeanSf)/(length(controller(it).epRobustness.N1234567)-1);
    ufSf1 = (controller(it).epRobustness.N1234567'*controller(it).epRobustness.N1)/(length(controller(it).epRobustness.N1234567)-1);
    ufSf2 = (controller(it).epRobustness.N1234567'*controller(it).epRobustness.N2)/(length(controller(it).epRobustness.N1234567)-1);
    ufSf3 = (controller(it).epRobustness.N1234567'*controller(it).epRobustness.N3)/(length(controller(it).epRobustness.N1234567)-1);
    ufSf4 = (controller(it).epRobustness.N1234567'*controller(it).epRobustness.N4)/(length(controller(it).epRobustness.N1234567)-1);
    ufSf5 = (controller(it).epRobustness.N1234567'*controller(it).epRobustness.N5)/(length(controller(it).epRobustness.N1234567)-1);
    ufSf6 = (controller(it).epRobustness.N1234567'*controller(it).epRobustness.N6)/(length(controller(it).epRobustness.N1234567)-1);
    ufSf7 = (controller(it).epRobustness.N1234567'*controller(it).epRobustness.N7)/(length(controller(it).epRobustness.N1234567)-1);
    controller(it).epRobustness.Sf1 = (ufSf1-squaredMeanSf)/variance;
    controller(it).epRobustness.Sf2 = (ufSf2-squaredMeanSf)/variance;
    controller(it).epRobustness.Sf3 = (ufSf3-squaredMeanSf)/variance;
    controller(it).epRobustness.Sf4 = (ufSf4-squaredMeanSf)/variance;
    controller(it).epRobustness.Sf5 = (ufSf5-squaredMeanSf)/variance;
    controller(it).epRobustness.Sf6 = (ufSf6-squaredMeanSf)/variance;
    controller(it).epRobustness.Sf7 = (ufSf7-squaredMeanSf)/variance;
    % STf index
    squaredMeanSTf = mean(controller(it).epRobustness.N0)^2;
    ufSTf1 = (controller(it).epRobustness.N0'*controller(it).epRobustness.N1)/(length(controller(it).epRobustness.N0)-1);
    ufSTf2 = (controller(it).epRobustness.N0'*controller(it).epRobustness.N2)/(length(controller(it).epRobustness.N0)-1);
    ufSTf3 = (controller(it).epRobustness.N0'*controller(it).epRobustness.N3)/(length(controller(it).epRobustness.N0)-1);
    ufSTf4 = (controller(it).epRobustness.N0'*controller(it).epRobustness.N4)/(length(controller(it).epRobustness.N0)-1);
    ufSTf5 = (controller(it).epRobustness.N0'*controller(it).epRobustness.N5)/(length(controller(it).epRobustness.N0)-1);
    ufSTf6 = (controller(it).epRobustness.N0'*controller(it).epRobustness.N6)/(length(controller(it).epRobustness.N0)-1);
    ufSTf7 = (controller(it).epRobustness.N0'*controller(it).epRobustness.N7)/(length(controller(it).epRobustness.N0)-1);
    controller(it).epRobustness.STf1 = 1-(ufSTf1-squaredMeanSTf)/variance;
    controller(it).epRobustness.STf2 = 1-(ufSTf2-squaredMeanSTf)/variance;
    controller(it).epRobustness.STf3 = 1-(ufSTf3-squaredMeanSTf)/variance;
    controller(it).epRobustness.STf4 = 1-(ufSTf4-squaredMeanSTf)/variance;
    controller(it).epRobustness.STf5 = 1-(ufSTf5-squaredMeanSTf)/variance;
    controller(it).epRobustness.STf6 = 1-(ufSTf6-squaredMeanSTf)/variance;
    controller(it).epRobustness.STf7 = 1-(ufSTf7-squaredMeanSTf)/variance;
    % SIf index
    controller(it).epRobustness.SIf1 = controller(it).epRobustness.STf1-controller(it).epRobustness.Sf1;
    controller(it).epRobustness.SIf2 = controller(it).epRobustness.STf2-controller(it).epRobustness.Sf2;
    controller(it).epRobustness.SIf3 = controller(it).epRobustness.STf3-controller(it).epRobustness.Sf3;
    controller(it).epRobustness.SIf4 = controller(it).epRobustness.STf4-controller(it).epRobustness.Sf4;
    controller(it).epRobustness.SIf5 = controller(it).epRobustness.STf5-controller(it).epRobustness.Sf5;
    controller(it).epRobustness.SIf6 = controller(it).epRobustness.STf6-controller(it).epRobustness.Sf6;
    controller(it).epRobustness.SIf7 = controller(it).epRobustness.STf7-controller(it).epRobustness.Sf7;
    %Variance    
    controller(it).epRobustness.variance = variance;
    % RMS Power
    % Sf index
    squaredMeanSf = mean(controller(it).PRobustness.N0.*controller(it).PRobustness.N1234567);
    variance = sum(controller(it).PRobustness.N1234567.^2.-squaredMeanSf)/(length(controller(it).PRobustness.N1234567)-1);
    ufSf1 = (controller(it).PRobustness.N1234567'*controller(it).PRobustness.N1)/(length(controller(it).PRobustness.N1234567)-1);
    ufSf2 = (controller(it).PRobustness.N1234567'*controller(it).PRobustness.N2)/(length(controller(it).PRobustness.N1234567)-1);
    ufSf3 = (controller(it).PRobustness.N1234567'*controller(it).PRobustness.N3)/(length(controller(it).PRobustness.N1234567)-1);
    ufSf4 = (controller(it).PRobustness.N1234567'*controller(it).PRobustness.N4)/(length(controller(it).PRobustness.N1234567)-1);
    ufSf5 = (controller(it).PRobustness.N1234567'*controller(it).PRobustness.N5)/(length(controller(it).PRobustness.N1234567)-1);
    ufSf6 = (controller(it).PRobustness.N1234567'*controller(it).PRobustness.N6)/(length(controller(it).PRobustness.N1234567)-1);
    ufSf7 = (controller(it).PRobustness.N1234567'*controller(it).PRobustness.N7)/(length(controller(it).PRobustness.N1234567)-1);
    controller(it).PRobustness.Sf1 = (ufSf1-squaredMeanSf)/variance;
    controller(it).PRobustness.Sf2 = (ufSf2-squaredMeanSf)/variance;
    controller(it).PRobustness.Sf3 = (ufSf3-squaredMeanSf)/variance;
    controller(it).PRobustness.Sf4 = (ufSf4-squaredMeanSf)/variance;
    controller(it).PRobustness.Sf5 = (ufSf5-squaredMeanSf)/variance;
    controller(it).PRobustness.Sf6 = (ufSf6-squaredMeanSf)/variance;
    controller(it).PRobustness.Sf7 = (ufSf7-squaredMeanSf)/variance;
    % STf index
    squaredMeanSTf = mean(controller(it).PRobustness.N0)^2;
    ufSTf1 = (controller(it).PRobustness.N0'*controller(it).PRobustness.N1)/(length(controller(it).PRobustness.N0)-1);
    ufSTf2 = (controller(it).PRobustness.N0'*controller(it).PRobustness.N2)/(length(controller(it).PRobustness.N0)-1);
    ufSTf3 = (controller(it).PRobustness.N0'*controller(it).PRobustness.N3)/(length(controller(it).PRobustness.N0)-1);
    ufSTf4 = (controller(it).PRobustness.N0'*controller(it).PRobustness.N4)/(length(controller(it).PRobustness.N0)-1);
    ufSTf5 = (controller(it).PRobustness.N0'*controller(it).PRobustness.N5)/(length(controller(it).PRobustness.N0)-1);
    ufSTf6 = (controller(it).PRobustness.N0'*controller(it).PRobustness.N6)/(length(controller(it).PRobustness.N0)-1);
    ufSTf7 = (controller(it).PRobustness.N0'*controller(it).PRobustness.N7)/(length(controller(it).PRobustness.N0)-1);
    controller(it).PRobustness.STf1 = 1-(ufSTf1-squaredMeanSTf)/variance;
    controller(it).PRobustness.STf2 = 1-(ufSTf2-squaredMeanSTf)/variance;
    controller(it).PRobustness.STf3 = 1-(ufSTf3-squaredMeanSTf)/variance;
    controller(it).PRobustness.STf4 = 1-(ufSTf4-squaredMeanSTf)/variance;
    controller(it).PRobustness.STf5 = 1-(ufSTf5-squaredMeanSTf)/variance;
    controller(it).PRobustness.STf6 = 1-(ufSTf6-squaredMeanSTf)/variance;
    controller(it).PRobustness.STf7 = 1-(ufSTf7-squaredMeanSTf)/variance;
    % SIf index
    controller(it).PRobustness.SIf1 = controller(it).PRobustness.STf1-controller(it).PRobustness.Sf1;
    controller(it).PRobustness.SIf2 = controller(it).PRobustness.STf2-controller(it).PRobustness.Sf2;
    controller(it).PRobustness.SIf3 = controller(it).PRobustness.STf3-controller(it).PRobustness.Sf3;
    controller(it).PRobustness.SIf4 = controller(it).PRobustness.STf4-controller(it).PRobustness.Sf4;
    controller(it).PRobustness.SIf5 = controller(it).PRobustness.STf5-controller(it).PRobustness.Sf5;
    controller(it).PRobustness.SIf6 = controller(it).PRobustness.STf6-controller(it).PRobustness.Sf6;
    controller(it).PRobustness.SIf7 = controller(it).PRobustness.STf7-controller(it).PRobustness.Sf7;
    %Variance    
    controller(it).PRobustness.variance = variance;
    % Simulation Success
    % Sf index
    squaredMeanSf = mean(controller(it).ssRobustness.N0.*controller(it).ssRobustness.N1234567);
    variance = sum(controller(it).ssRobustness.N1234567.^2.-squaredMeanSf)/(length(controller(it).ssRobustness.N1234567)-1);
    ufSf1 = (controller(it).ssRobustness.N1234567'*controller(it).ssRobustness.N1)/(length(controller(it).ssRobustness.N1234567)-1);
    ufSf2 = (controller(it).ssRobustness.N1234567'*controller(it).ssRobustness.N2)/(length(controller(it).ssRobustness.N1234567)-1);
    ufSf3 = (controller(it).ssRobustness.N1234567'*controller(it).ssRobustness.N3)/(length(controller(it).ssRobustness.N1234567)-1);
    ufSf4 = (controller(it).ssRobustness.N1234567'*controller(it).ssRobustness.N4)/(length(controller(it).ssRobustness.N1234567)-1);
    ufSf5 = (controller(it).ssRobustness.N1234567'*controller(it).ssRobustness.N5)/(length(controller(it).ssRobustness.N1234567)-1);
    ufSf6 = (controller(it).ssRobustness.N1234567'*controller(it).ssRobustness.N6)/(length(controller(it).ssRobustness.N1234567)-1);
    ufSf7 = (controller(it).ssRobustness.N1234567'*controller(it).ssRobustness.N7)/(length(controller(it).ssRobustness.N1234567)-1);
    controller(it).ssRobustness.Sf1 = (ufSf1-squaredMeanSf)/variance;
    controller(it).ssRobustness.Sf2 = (ufSf2-squaredMeanSf)/variance;
    controller(it).ssRobustness.Sf3 = (ufSf3-squaredMeanSf)/variance;
    controller(it).ssRobustness.Sf4 = (ufSf4-squaredMeanSf)/variance;
    controller(it).ssRobustness.Sf5 = (ufSf5-squaredMeanSf)/variance;
    controller(it).ssRobustness.Sf6 = (ufSf6-squaredMeanSf)/variance;
    controller(it).ssRobustness.Sf7 = (ufSf7-squaredMeanSf)/variance;
    % STf index
    squaredMeanSTf = mean(controller(it).ssRobustness.N0)^2;
    ufSTf1 = (controller(it).ssRobustness.N0'*controller(it).ssRobustness.N1)/(length(controller(it).ssRobustness.N0)-1);
    ufSTf2 = (controller(it).ssRobustness.N0'*controller(it).ssRobustness.N2)/(length(controller(it).ssRobustness.N0)-1);
    ufSTf3 = (controller(it).ssRobustness.N0'*controller(it).ssRobustness.N3)/(length(controller(it).ssRobustness.N0)-1);
    ufSTf4 = (controller(it).ssRobustness.N0'*controller(it).ssRobustness.N4)/(length(controller(it).ssRobustness.N0)-1);
    ufSTf5 = (controller(it).ssRobustness.N0'*controller(it).ssRobustness.N5)/(length(controller(it).ssRobustness.N0)-1);
    ufSTf6 = (controller(it).ssRobustness.N0'*controller(it).ssRobustness.N6)/(length(controller(it).ssRobustness.N0)-1);
    ufSTf7 = (controller(it).ssRobustness.N0'*controller(it).ssRobustness.N7)/(length(controller(it).ssRobustness.N0)-1);
    controller(it).ssRobustness.STf1 = 1-(ufSTf1-squaredMeanSTf)/variance;
    controller(it).ssRobustness.STf2 = 1-(ufSTf2-squaredMeanSTf)/variance;
    controller(it).ssRobustness.STf3 = 1-(ufSTf3-squaredMeanSTf)/variance;
    controller(it).ssRobustness.STf4 = 1-(ufSTf4-squaredMeanSTf)/variance;
    controller(it).ssRobustness.STf5 = 1-(ufSTf5-squaredMeanSTf)/variance;
    controller(it).ssRobustness.STf6 = 1-(ufSTf6-squaredMeanSTf)/variance;
    controller(it).ssRobustness.STf7 = 1-(ufSTf7-squaredMeanSTf)/variance;
    % SIf index
    controller(it).ssRobustness.SIf1 = controller(it).ssRobustness.STf1-controller(it).ssRobustness.Sf1;
    controller(it).ssRobustness.SIf2 = controller(it).ssRobustness.STf2-controller(it).ssRobustness.Sf2;
    controller(it).ssRobustness.SIf3 = controller(it).ssRobustness.STf3-controller(it).ssRobustness.Sf3;
    controller(it).ssRobustness.SIf4 = controller(it).ssRobustness.STf4-controller(it).ssRobustness.Sf4;
    controller(it).ssRobustness.SIf5 = controller(it).ssRobustness.STf5-controller(it).ssRobustness.Sf5;
    controller(it).ssRobustness.SIf6 = controller(it).ssRobustness.STf6-controller(it).ssRobustness.Sf6;
    controller(it).ssRobustness.SIf7 = controller(it).ssRobustness.STf7-controller(it).ssRobustness.Sf7;
    %Variance    
    controller(it).ssRobustness.variance = variance;   
    disp(['Finished controller ',controller(it).name]);
    end
end
uisave('controller','evaluationMetrics.mat');
clear all