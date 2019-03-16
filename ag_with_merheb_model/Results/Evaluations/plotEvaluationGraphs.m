clear all

% Get a list of all files and folders in this folder.
files = dir(pwd);
% Get a logical vector that tells which is a directory.
dirFlags = [files.isdir];
% Extract only those that are directories.
subFolders = files(dirFlags);
subFolders(1:2) = [];

controller = struct([]);

for it=1:length(subFolders)
    aux = strsplit(subFolders(it).name,'_');
    % Get controller name from folder name
    controller(it).name = aux(2);
    % Load evaluation data
    data = load([subFolders(it).folder,'/',subFolders(it).name,'/evaluationResult.mat'],'options');  
    controller(it).data = data.options;
    %% Overall sucess rate
    controller(it).metrics = [controller(it).data{:,8}];
    numberOfSimulations = length(controller(it).metrics);
    numberOfSuccesses = sum([controller(it).metrics.simulationSuccess] == 1);
    numberOfFailures = numberOfSimulations-numberOfSuccesses;
    controller(it).overallSuccessRate = numberOfSuccesses/numberOfSimulations;
    %% Overall mean position error
    indexSuccesful = find([controller(it).metrics.simulationSuccess] == 1);
    indexFail = find([controller(it).metrics.simulationSuccess] ~= 1);
    rmsPositionError = [controller(it).metrics.RMSPositionError]';
    rmsPower = [controller(it).metrics.RMSPower]';
    controller(it).successMeanError = mean(rmsPositionError(indexSuccesful));
    controller(it).failMeanError = mean(rmsPositionError(indexFail));
    controller(it).meanError = mean(rmsPositionError);
    %% Overall mean power
    controller(it).successMeanPower = mean(rmsPower(indexSuccesful));
    controller(it).failMeanPower = mean(rmsPower(indexFail));
    controller(it).meanPower = mean(rmsPower);
    
    %% Compound analysis
    endTimes = sort(unique([controller(it).data{:,1}])','descend');
    disturbances = unique([controller(it).data{:,4}])';
    payloads = sortrows(unique(cell2mat(controller(it).data(:,3)),'rows'));
    controlLoops = unique([controller(it).data{:,6}])';
    simulationSuccess = [controller(it).metrics.simulationSuccess]';
    rmsPositionError = [controller(it).metrics.RMSPositionError]';
    rmsPower = [controller(it).metrics.RMSPower]';
    
    aux = controller(it).data(:,5);
    aux1 = "";
    for jt=1:length(aux)
        aux1(jt,1) = strjoin(string(aux{jt}),'|');
    end
    allFailures = aux1;
    failures = unique(aux1);
    %% Find indexes
    index0EndTimes = ([controller(it).data{:,1}] == endTimes(1))';
    index0Disturbances = ([controller(it).data{:,4}] == disturbances(1))';
    aux = [controller(it).data{:,3}];
    aux = reshape(aux,4,length(aux)/4)';
    index0Payloads = ismember(aux,payloads(1,:),'rows');
    index0ControlLoops = ([controller(it).data{:,6}] == controlLoops(1))';
    index0Failures = (allFailures == "");
    % Compound indexes
    indexPayloadEndTimes = find(index0Disturbances & index0ControlLoops & index0Failures);
    indexPayloadDisturbances = find(index0EndTimes & index0ControlLoops & index0Failures);
    indexPayloadFailures = find(index0Disturbances & index0ControlLoops & index0EndTimes);
    indexEndTimesDisturbances = find(index0Payloads & index0ControlLoops & index0Failures);
    indexEndTimesFailures = find(index0Payloads & index0ControlLoops & index0Disturbances);
    indexFailuresDisturbances = find(index0Payloads & index0ControlLoops & index0EndTimes);
    
    % Payload X endTimes 
    thisTestIndexes = indexPayloadEndTimes;
    testSimulationSuccess = simulationSuccess(thisTestIndexes);
    testRMSPositionError = rmsPositionError(thisTestIndexes);
    testRMSPower = rmsPower(thisTestIndexes);
    
    indexSuccesful = find(testSimulationSuccess == 1);
    indexFail = find(testSimulationSuccess ~= 1);
    controller(it).payloadEndTimes.successRate = length(indexSuccesful)/length(testSimulationSuccess);
    controller(it).payloadEndTimes.successMeanError = mean(testRMSPositionError(indexSuccesful));
    controller(it).payloadEndTimes.failMeanError = mean(testRMSPositionError(indexFail));
    controller(it).payloadEndTimes.meanError = mean(testRMSPositionError);
    controller(it).payloadEndTimes.successMeanPower = mean(testRMSPower(indexSuccesful));
    controller(it).payloadEndTimes.failMeanPower = mean(testRMSPower(indexFail));
    controller(it).payloadEndTimes.meanPower = mean(testRMSPower);
    
    % Payload X Disturbances 
    thisTestIndexes = indexPayloadDisturbances;
    testSimulationSuccess = simulationSuccess(thisTestIndexes);
    testRMSPositionError = rmsPositionError(thisTestIndexes);
    testRMSPower = rmsPower(thisTestIndexes);
    
    indexSuccesful = find(testSimulationSuccess == 1);
    indexFail = find(testSimulationSuccess ~= 1);
    controller(it).payloadDisturbances.successRate = length(indexSuccesful)/length(testSimulationSuccess);
    controller(it).payloadDisturbances.successMeanError = mean(testRMSPositionError(indexSuccesful));
    controller(it).payloadDisturbances.failMeanError = mean(testRMSPositionError(indexFail));
    controller(it).payloadDisturbances.meanError = mean(testRMSPositionError);
    controller(it).payloadDisturbances.successMeanPower = mean(testRMSPower(indexSuccesful));
    controller(it).payloadDisturbances.failMeanPower = mean(testRMSPower(indexFail));
    controller(it).payloadDisturbances.meanPower = mean(testRMSPower);
    
    % Payload X Failures 
    thisTestIndexes = indexPayloadFailures;
    testSimulationSuccess = simulationSuccess(thisTestIndexes);
    testRMSPositionError = rmsPositionError(thisTestIndexes);
    testRMSPower = rmsPower(thisTestIndexes);
    
    indexSuccesful = find(testSimulationSuccess == 1);
    indexFail = find(testSimulationSuccess ~= 1);
    controller(it).payloadFailures.successRate = length(indexSuccesful)/length(testSimulationSuccess);
    controller(it).payloadFailures.successMeanError = mean(testRMSPositionError(indexSuccesful));
    controller(it).payloadFailures.failMeanError = mean(testRMSPositionError(indexFail));
    controller(it).payloadFailures.meanError = mean(testRMSPositionError);
    controller(it).payloadFailures.successMeanPower = mean(testRMSPower(indexSuccesful));
    controller(it).payloadFailures.failMeanPower = mean(testRMSPower(indexFail));
    controller(it).payloadFailures.meanPower = mean(testRMSPower);
    
    % EndTimes X Disturbances
    thisTestIndexes = indexEndTimesDisturbances;
    testSimulationSuccess = simulationSuccess(thisTestIndexes);
    testRMSPositionError = rmsPositionError(thisTestIndexes);
    testRMSPower = rmsPower(thisTestIndexes);
    
    indexSuccesful = find(testSimulationSuccess == 1);
    indexFail = find(testSimulationSuccess ~= 1);
    controller(it).endTimesDisturbances.successRate = length(indexSuccesful)/length(testSimulationSuccess);
    controller(it).endTimesDisturbances.successMeanError = mean(testRMSPositionError(indexSuccesful));
    controller(it).endTimesDisturbances.failMeanError = mean(testRMSPositionError(indexFail));
    controller(it).endTimesDisturbances.meanError = mean(testRMSPositionError);
    controller(it).endTimesDisturbances.successMeanPower = mean(testRMSPower(indexSuccesful));
    controller(it).endTimesDisturbances.failMeanPower = mean(testRMSPower(indexFail));
    controller(it).endTimesDisturbances.meanPower = mean(testRMSPower);
    
    % EndTimes X Failures
    thisTestIndexes = indexEndTimesFailures;
    testSimulationSuccess = simulationSuccess(thisTestIndexes);
    testRMSPositionError = rmsPositionError(thisTestIndexes);
    testRMSPower = rmsPower(thisTestIndexes);
    
    indexSuccesful = find(testSimulationSuccess == 1);
    indexFail = find(testSimulationSuccess ~= 1);
    controller(it).endTimesFailures.successRate = length(indexSuccesful)/length(testSimulationSuccess);
    controller(it).endTimesFailures.successMeanError = mean(testRMSPositionError(indexSuccesful));
    controller(it).endTimesFailures.failMeanError = mean(testRMSPositionError(indexFail));
    controller(it).endTimesFailures.meanError = mean(testRMSPositionError);
    controller(it).endTimesFailures.successMeanPower = mean(testRMSPower(indexSuccesful));
    controller(it).endTimesFailures.failMeanPower = mean(testRMSPower(indexFail));
    controller(it).endTimesFailures.meanPower = mean(testRMSPower);
    
    % Failures X Disturbances
    thisTestIndexes = indexFailuresDisturbances;
    testSimulationSuccess = simulationSuccess(thisTestIndexes);
    testRMSPositionError = rmsPositionError(thisTestIndexes);
    testRMSPower = rmsPower(thisTestIndexes);
    
    indexSuccesful = find(testSimulationSuccess == 1);
    indexFail = find(testSimulationSuccess ~= 1);
    controller(it).failuresDisturbances.successRate = length(indexSuccesful)/length(testSimulationSuccess);
    controller(it).failuresDisturbances.successMeanError = mean(testRMSPositionError(indexSuccesful));
    controller(it).failuresDisturbances.failMeanError = mean(testRMSPositionError(indexFail));
    controller(it).failuresDisturbances.meanError = mean(testRMSPositionError);
    controller(it).failuresDisturbances.successMeanPower = mean(testRMSPower(indexSuccesful));
    controller(it).failuresDisturbances.failMeanPower = mean(testRMSPower(indexFail));
    controller(it).failuresDisturbances.meanPower = mean(testRMSPower);
    
    disp(['Finished controller ',controller(it).name]);
end

%% Compare all controllers
%% Overall tests
% Overall success rate
figure
names = [];
values = [];
for it=1:length(controller)
    names = [names; controller(it).name];
    values = [values; controller(it).overallSuccessRate];
end
nameCategories = categorical(names);
nameCategories = reordercats(nameCategories,names);
bar(nameCategories,values)

% Overall success mean error
figure
values = [];
for it=1:length(controller)
    values = [values; controller(it).successMeanError];
end
bar(nameCategories,values)

% Overall success mean power
figure
values = [];
for it=1:length(controller)
    values = [values; controller(it).successMeanPower];
end
bar(nameCategories,values)

%% Compound tests
% Success rate
figure
values = [];
for it=1:length(controller)
    values = [values; controller(it).payloadEndTimes.successRate controller(it).payloadDisturbances.successRate controller(it).payloadFailures.successRate controller(it).endTimesDisturbances.successRate controller(it).endTimesFailures.successRate controller(it).failuresDisturbances.successRate];
end
plot(nameCategories,values)
legend('Payload X endTimes','Payload X Disturbances','Payload X Failures','EndTimes X Disturbances','EndTimes X Failures','Failures X Disturbances');
% Success mean error
figure
values = [];
for it=1:length(controller)
    values = [values; controller(it).payloadEndTimes.successMeanError controller(it).payloadDisturbances.successMeanError controller(it).payloadFailures.successMeanError controller(it).endTimesDisturbances.successMeanError controller(it).endTimesFailures.successMeanError controller(it).failuresDisturbances.successMeanError];
end
plot(nameCategories,values)
legend('Payload X endTimes','Payload X Disturbances','Payload X Failures','EndTimes X Disturbances','EndTimes X Failures','Failures X Disturbances');
% Success mean power
figure
values = [];
for it=1:length(controller)
    values = [values; controller(it).payloadEndTimes.successMeanPower controller(it).payloadDisturbances.successMeanPower controller(it).payloadFailures.successMeanPower controller(it).endTimesDisturbances.successMeanPower controller(it).endTimesFailures.successMeanPower controller(it).failuresDisturbances.successMeanPower];
end
plot(nameCategories,values)
legend('Payload X endTimes','Payload X Disturbances','Payload X Failures','EndTimes X Disturbances','EndTimes X Failures','Failures X Disturbances');


% %% Payload X EndTimes
% % Success rate
% figure
% values = [];
% for it=1:length(controller)
%     values = [values; controller(it).payloadEndTimes.successRate];
% end
% bar(categorical(names),values)
% % Success mean error
% figure
% values = [];
% for it=1:length(controller)
%     values = [values; controller(it).payloadEndTimes.successMeanError];
% end
% bar(categorical(names),values)
% % Success mean power
% figure
% values = [];
% for it=1:length(controller)
%     values = [values; controller(it).payloadEndTimes.successMeanPower];
% end
% bar(categorical(names),values)
% 
% %% Payload X Disturbances
% % Success rate
% figure
% values = [];
% for it=1:length(controller)
%     values = [values; controller(it).payloadDisturbances.successRate];
% end
% bar(categorical(names),values)
% % Success mean error
% figure
% values = [];
% for it=1:length(controller)
%     values = [values; controller(it).payloadDisturbances.successMeanError];
% end
% bar(categorical(names),values)
% % Success mean power
% figure
% values = [];
% for it=1:length(controller)
%     values = [values; controller(it).payloadDisturbances.successMeanPower];
% end
% bar(categorical(names),values)
% 
% %% Payload X Failures
% % Success rate
% figure
% values = [];
% for it=1:length(controller)
%     values = [values; controller(it).payloadFailures.successRate];
% end
% bar(categorical(names),values)
% % Success mean error
% figure
% values = [];
% for it=1:length(controller)
%     values = [values; controller(it).payloadFailures.successMeanError];
% end
% bar(categorical(names),values)
% % Success mean power
% figure
% values = [];
% for it=1:length(controller)
%     values = [values; controller(it).payloadFailures.successMeanPower];
% end
% bar(categorical(names),values)
% 
% %% EndTimes X Disturbances
% % Success rate
% figure
% values = [];
% for it=1:length(controller)
%     values = [values; controller(it).endTimesDisturbances.successRate];
% end
% bar(categorical(names),values)
% % Success mean error
% figure
% values = [];
% for it=1:length(controller)
%     values = [values; controller(it).endTimesDisturbances.successMeanError];
% end
% bar(categorical(names),values)
% % Success mean power
% figure
% values = [];
% for it=1:length(controller)
%     values = [values; controller(it).endTimesDisturbances.successMeanPower];
% end
% bar(categorical(names),values)
% 
% %% EndTimes X Failures
% % Success rate
% figure
% values = [];
% for it=1:length(controller)
%     values = [values; controller(it).endTimesFailures.successRate];
% end
% bar(categorical(names),values)
% % Success mean error
% figure
% values = [];
% for it=1:length(controller)
%     values = [values; controller(it).endTimesFailures.successMeanError];
% end
% bar(categorical(names),values)
% % Success mean power
% figure
% values = [];
% for it=1:length(controller)
%     values = [values; controller(it).endTimesFailures.successMeanPower];
% end
% bar(categorical(names),values)
% 
% %% Failures X Disturbances
% % Success rate
% figure
% values = [];
% for it=1:length(controller)
%     values = [values; controller(it).failuresDisturbances.successRate];
% end
% bar(categorical(names),values)
% % Success mean error
% figure
% values = [];
% for it=1:length(controller)
%     values = [values; controller(it).failuresDisturbances.successMeanError];
% end
% bar(categorical(names),values)
% % Success mean power
% figure
% values = [];
% for it=1:length(controller)
%     values = [values; controller(it).failuresDisturbances.successMeanPower];
% end
% bar(categorical(names),values)
