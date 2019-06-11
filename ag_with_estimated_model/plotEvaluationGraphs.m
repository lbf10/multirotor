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
                    %aux = 400/metrics(kt).simulationSuccess;
                    aux = metrics(kt).RMSPower;
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
    
    
%%
%     
%     %% Compound analysis
%     endTimes = sort(unique([controller(it).data{:,1}])','descend');
%     disturbances = unique([controller(it).data{:,4}])';
%     payloads = sortrows(unique(cell2mat(controller(it).data(:,3)),'rows'));
%     controlLoops = unique([controller(it).data{:,6}])';
%     simulationSuccess = [controller(it).metrics.simulationSuccess]';
%     rmsPositionError = [controller(it).metrics.RMSPositionError]';
%     rmsPower = [controller(it).metrics.RMSPower]';
%     
%     aux = controller(it).data(:,5);
%     aux1 = "";
%     for jt=1:length(aux)
%         aux1(jt,1) = strjoin(string(aux{jt}),'|');
%     end
%     allFailures = aux1;
%     failures = unique(aux1);
%     %% Find indexes
%     index0EndTimes = ([controller(it).data{:,1}] == endTimes(1))';
%     index0Disturbances = ([controller(it).data{:,4}] == disturbances(1))';
%     aux = [controller(it).data{:,3}];
%     aux = reshape(aux,4,length(aux)/4)';
%     index0Payloads = ismember(aux,payloads(1,:),'rows');
%     index0ControlLoops = ([controller(it).data{:,6}] == controlLoops(1))';
%     index0Failures = (allFailures == "");
%     % Compound indexes
%     indexPayloadEndTimes = find(index0Disturbances & index0ControlLoops & index0Failures);
%     indexPayloadDisturbances = find(index0EndTimes & index0ControlLoops & index0Failures);
%     indexPayloadFailures = find(index0Disturbances & index0ControlLoops & index0EndTimes);
%     indexEndTimesDisturbances = find(index0Payloads & index0ControlLoops & index0Failures);
%     indexEndTimesFailures = find(index0Payloads & index0ControlLoops & index0Disturbances);
%     indexFailuresDisturbances = find(index0Payloads & index0ControlLoops & index0EndTimes);
%     
%     % Payload X endTimes 
%     thisTestIndexes = indexPayloadEndTimes;
%     testSimulationSuccess = simulationSuccess(thisTestIndexes);
%     testRMSPositionError = rmsPositionError(thisTestIndexes);
%     testRMSPower = rmsPower(thisTestIndexes);
%     
%     indexSuccesful = find(testSimulationSuccess == 1);
%     indexFail = find(testSimulationSuccess ~= 1);
%     controller(it).payloadEndTimes.successRate = length(indexSuccesful)/length(testSimulationSuccess);
%     controller(it).payloadEndTimes.successMeanError = mean(testRMSPositionError(indexSuccesful));
%     controller(it).payloadEndTimes.failMeanError = mean(testRMSPositionError(indexFail));
%     controller(it).payloadEndTimes.meanError = mean(testRMSPositionError);
%     controller(it).payloadEndTimes.successMeanPower = mean(testRMSPower(indexSuccesful));
%     controller(it).payloadEndTimes.failMeanPower = mean(testRMSPower(indexFail));
%     controller(it).payloadEndTimes.meanPower = mean(testRMSPower);
%     
%     % Payload X Disturbances 
%     thisTestIndexes = indexPayloadDisturbances;
%     testSimulationSuccess = simulationSuccess(thisTestIndexes);
%     testRMSPositionError = rmsPositionError(thisTestIndexes);
%     testRMSPower = rmsPower(thisTestIndexes);
%     
%     indexSuccesful = find(testSimulationSuccess == 1);
%     indexFail = find(testSimulationSuccess ~= 1);
%     controller(it).payloadDisturbances.successRate = length(indexSuccesful)/length(testSimulationSuccess);
%     controller(it).payloadDisturbances.successMeanError = mean(testRMSPositionError(indexSuccesful));
%     controller(it).payloadDisturbances.failMeanError = mean(testRMSPositionError(indexFail));
%     controller(it).payloadDisturbances.meanError = mean(testRMSPositionError);
%     controller(it).payloadDisturbances.successMeanPower = mean(testRMSPower(indexSuccesful));
%     controller(it).payloadDisturbances.failMeanPower = mean(testRMSPower(indexFail));
%     controller(it).payloadDisturbances.meanPower = mean(testRMSPower);
%     
%     % Payload X Failures 
%     thisTestIndexes = indexPayloadFailures;
%     testSimulationSuccess = simulationSuccess(thisTestIndexes);
%     testRMSPositionError = rmsPositionError(thisTestIndexes);
%     testRMSPower = rmsPower(thisTestIndexes);
%     
%     indexSuccesful = find(testSimulationSuccess == 1);
%     indexFail = find(testSimulationSuccess ~= 1);
%     controller(it).payloadFailures.successRate = length(indexSuccesful)/length(testSimulationSuccess);
%     controller(it).payloadFailures.successMeanError = mean(testRMSPositionError(indexSuccesful));
%     controller(it).payloadFailures.failMeanError = mean(testRMSPositionError(indexFail));
%     controller(it).payloadFailures.meanError = mean(testRMSPositionError);
%     controller(it).payloadFailures.successMeanPower = mean(testRMSPower(indexSuccesful));
%     controller(it).payloadFailures.failMeanPower = mean(testRMSPower(indexFail));
%     controller(it).payloadFailures.meanPower = mean(testRMSPower);
%     
%     % EndTimes X Disturbances
%     thisTestIndexes = indexEndTimesDisturbances;
%     testSimulationSuccess = simulationSuccess(thisTestIndexes);
%     testRMSPositionError = rmsPositionError(thisTestIndexes);
%     testRMSPower = rmsPower(thisTestIndexes);
%     
%     indexSuccesful = find(testSimulationSuccess == 1);
%     indexFail = find(testSimulationSuccess ~= 1);
%     controller(it).endTimesDisturbances.successRate = length(indexSuccesful)/length(testSimulationSuccess);
%     controller(it).endTimesDisturbances.successMeanError = mean(testRMSPositionError(indexSuccesful));
%     controller(it).endTimesDisturbances.failMeanError = mean(testRMSPositionError(indexFail));
%     controller(it).endTimesDisturbances.meanError = mean(testRMSPositionError);
%     controller(it).endTimesDisturbances.successMeanPower = mean(testRMSPower(indexSuccesful));
%     controller(it).endTimesDisturbances.failMeanPower = mean(testRMSPower(indexFail));
%     controller(it).endTimesDisturbances.meanPower = mean(testRMSPower);
%     
%     % EndTimes X Failures
%     thisTestIndexes = indexEndTimesFailures;
%     testSimulationSuccess = simulationSuccess(thisTestIndexes);
%     testRMSPositionError = rmsPositionError(thisTestIndexes);
%     testRMSPower = rmsPower(thisTestIndexes);
%     
%     indexSuccesful = find(testSimulationSuccess == 1);
%     indexFail = find(testSimulationSuccess ~= 1);
%     controller(it).endTimesFailures.successRate = length(indexSuccesful)/length(testSimulationSuccess);
%     controller(it).endTimesFailures.successMeanError = mean(testRMSPositionError(indexSuccesful));
%     controller(it).endTimesFailures.failMeanError = mean(testRMSPositionError(indexFail));
%     controller(it).endTimesFailures.meanError = mean(testRMSPositionError);
%     controller(it).endTimesFailures.successMeanPower = mean(testRMSPower(indexSuccesful));
%     controller(it).endTimesFailures.failMeanPower = mean(testRMSPower(indexFail));
%     controller(it).endTimesFailures.meanPower = mean(testRMSPower);
%     
%     % Failures X Disturbances
%     thisTestIndexes = indexFailuresDisturbances;
%     testSimulationSuccess = simulationSuccess(thisTestIndexes);
%     testRMSPositionError = rmsPositionError(thisTestIndexes);
%     testRMSPower = rmsPower(thisTestIndexes);
%     
%     indexSuccesful = find(testSimulationSuccess == 1);
%     indexFail = find(testSimulationSuccess ~= 1);
%     controller(it).failuresDisturbances.successRate = length(indexSuccesful)/length(testSimulationSuccess);
%     controller(it).failuresDisturbances.successMeanError = mean(testRMSPositionError(indexSuccesful));
%     controller(it).failuresDisturbances.failMeanError = mean(testRMSPositionError(indexFail));
%     controller(it).failuresDisturbances.meanError = mean(testRMSPositionError);
%     controller(it).failuresDisturbances.successMeanPower = mean(testRMSPower(indexSuccesful));
%     controller(it).failuresDisturbances.failMeanPower = mean(testRMSPower(indexFail));
%     controller(it).failuresDisturbances.meanPower = mean(testRMSPower);
    
    disp(['Finished controller ',controller(it).name]);
end

% %% Compare all controllers
% %% Overall tests
% % Overall success rate
% figure
% names = [];
% values = [];
% for it=1:length(controller)
%     names = [names; controller(it).name];
%     values = [values; controller(it).overallSuccessRate];
% end
% nameCategories = categorical(names);
% nameCategories = reordercats(nameCategories,names);
% bar(nameCategories,values)
% title('Overall success rate')

% % Overall success mean error
% figure
% values = [];
% for it=1:length(controller)
%     values = [values; controller(it).successMeanError];
% end
% bar(nameCategories,values)
% title('Overall success mean error')
% 
% % Overall success mean power
% figure
% values = [];
% for it=1:length(controller)
%     values = [values; controller(it).successMeanPower];
% end
% bar(nameCategories,values)
% title('Overall success mean power')

% %% Compound tests
% % Success rate
% figure
% values = [];
% for it=1:length(controller)
%     values = [values; controller(it).payloadEndTimes.successRate controller(it).payloadDisturbances.successRate controller(it).payloadFailures.successRate controller(it).endTimesDisturbances.successRate controller(it).endTimesFailures.successRate controller(it).failuresDisturbances.successRate];
% end
% plot(nameCategories,values)
% legend('Payload X endTimes','Payload X Disturbances','Payload X Failures','EndTimes X Disturbances','EndTimes X Failures','Failures X Disturbances');
% title('Success rate')
% % Success mean error
% figure
% values = [];
% for it=1:length(controller)
%     values = [values; controller(it).payloadEndTimes.successMeanError controller(it).payloadDisturbances.successMeanError controller(it).payloadFailures.successMeanError controller(it).endTimesDisturbances.successMeanError controller(it).endTimesFailures.successMeanError controller(it).failuresDisturbances.successMeanError];
% end
% plot(nameCategories,values)
% legend('Payload X endTimes','Payload X Disturbances','Payload X Failures','EndTimes X Disturbances','EndTimes X Failures','Failures X Disturbances');
% title('Success mean error')
% % Success mean power
% figure
% values = [];
% for it=1:length(controller)
%     values = [values; controller(it).payloadEndTimes.successMeanPower controller(it).payloadDisturbances.successMeanPower controller(it).payloadFailures.successMeanPower controller(it).endTimesDisturbances.successMeanPower controller(it).endTimesFailures.successMeanPower controller(it).failuresDisturbances.successMeanPower];
% end
% plot(nameCategories,values)
% legend('Payload X endTimes','Payload X Disturbances','Payload X Failures','EndTimes X Disturbances','EndTimes X Failures','Failures X Disturbances');
% title('Success mean power')
