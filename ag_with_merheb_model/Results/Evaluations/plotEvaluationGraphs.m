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
    %% Mean overall fitness
    controller(it).meanOverallFitness = mean([controller(it).data{:,7}]);
    
    %% Fitness by failure type and difficulty
    % Converts and lists failure types
    aux = controller(it).data(:,5);
    aux1 = "";
    for jt=1:length(aux)
        aux1(jt,1) = strjoin(string(aux{jt}),'|');
    end
    controller(it).failures.id = aux1;
    controller(it).failures.types = unique(aux1);
    % Fitness per failure type
    fitness = [];
    for jt=1:length(controller(it).failures.types)
        typeIndex = find(controller(it).failures.id==controller(it).failures.types(jt));
        fitness(jt) = mean([controller(it).data{typeIndex,7}]);
    end
    controller(it).failures.typeFitness = fitness;
    % Classify failure types according to difficulty
    difficulty = [];
    for jt=1:length(controller(it).failures.types)
        difficulty(jt) = 0;
        failures = strsplit(controller(it).failures.types(jt),'|');
        failureData = {};
        if ~strcmp(failures,"")
            for kt=failures
                failureData(end+1,:) = textscan(kt,'setRotorStatus(%d,''motor loss'',%f)');
            end
            numberOfFailures = size(failureData,1);
            failuresOnPlane1 = sum([failureData{:,1}]>4);
            multiplier = 1.1^(1-abs((0.5-failuresOnPlane1/numberOfFailures)/0.5));
            sumIntensities = sum(1-[failureData{:,2}]);
            difficulty(jt) = multiplier*sumIntensities;
        end
    end
    [difficulty, index] = sort(difficulty);
    controller(it).failures.difficulties = difficulty;
    controller(it).failures.typeFitness = controller(it).failures.typeFitness(index);
    controller(it).failures.types = controller(it).failures.types(index);
    
    %% Fitness by test case difficulty (endTime, disturbance and payload)
    % Calculates tests difficulties
    endTimes = sort(unique([controller(it).data{:,1}])','descend');
    disturbances = unique([controller(it).data{:,4}])';
    payloads = sortrows(unique(cell2mat(controller(it).data(:,3)),'rows'));
    controlLoops = unique([controller(it).data{:,6}])';
    
    difficulty = [];
    for jt=1:length(controller(it).data(:,1))
        [~,ia,~] = intersect(payloads,controller(it).data{jt,3},'rows');
        difficulty(jt) = 3^(find(endTimes==controller(it).data{jt,1})-1)+3^(find(disturbances==controller(it).data{jt,4})-0.9)+3^(ia-0.8);
    end
    controller(it).tests.difficulties = difficulty';
    controller(it).tests.types = unique(difficulty');
    % Fitness per test type
    fitness = [];
    for jt=1:length(controller(it).tests.types)
        testIndex = find(controller(it).tests.difficulties==controller(it).tests.types(jt));
        fitness(jt) = mean([controller(it).data{testIndex,7}]);
    end
    controller(it).tests.typeFitness = fitness';
    
    %% Fitness by endTime
    fitness = [];
    for jt=1:length(endTimes)
        categoryIndex = find([controller(it).data{:,1}]==endTimes(jt));
        fitness(jt) = mean([controller(it).data{categoryIndex,7}]);
    end
    controller(it).endTimes = endTimes;
    controller(it).fitnessPerEndTime = fitness;
    %% Fitness by disturbance
    fitness = [];
    for jt=1:length(disturbances)
        categoryIndex = find([controller(it).data{:,4}]==disturbances(jt));
        fitness(jt) = mean([controller(it).data{categoryIndex,7}]);
    end
    controller(it).disturbances = disturbances;
    controller(it).fitnessPerDisturbance = fitness;
    %% Fitness by controlLoopTime
    fitness = [];
    for jt=1:length(controlLoops)
        categoryIndex = find([controller(it).data{:,6}]==controlLoops(jt));
        fitness(jt) = mean([controller(it).data{categoryIndex,7}]);
    end
    controller(it).controlLoops = controlLoops;
    controller(it).fitnessPerControlLoopTime = fitness;
    %% Fitness by payload
    fitness = [];
    aux = [controller(it).data{:,3}];
    aux = reshape(aux,4,length(aux)/4)';
    for jt=1:size(payloads,1)
        categoryIndex = find(ismember(aux,payloads(jt,:),'rows'));
        fitness(jt) = mean([controller(it).data{categoryIndex,7}]);
    end
    controller(it).payloads = payloads;
    controller(it).fitnessPerPayload = fitness;
    disp(['Finished controller ',controller(it).name]);
end

%% Compare all controllers
% Overal fitness
figure
names = [];
values = [];
for it=1:length(controller)
    names = [names; controller(it).name];
    values = [values; controller(it).meanOverallFitness];
end
bar(categorical(names),values)
% Failure types
figure
values = [];
for it=1:length(controller)
    values = controller(it).failures.typeFitness;
    plot(values,'color',rand(1,3))
    hold on
end
% Test types
figure
values = [];
for it=1:length(controller)
    values = controller(it).tests.typeFitness;
    plot(values,'color',rand(1,3))
    hold on
end
% endTimes
figure
endTimes = [];
values = [];
for it=1:length(controller)
    endTimes = controller(it).endTimes;
    values = controller(it).fitnessPerEndTime;
    plot(endTimes,values,'color',rand(1,3))
    hold on
end
% endTimes
figure
endTimes = [];
values = [];
for it=1:length(controller)
    endTimes = controller(it).endTimes;
    values = controller(it).fitnessPerEndTime;
    plot(endTimes,values,'color',rand(1,3))
    hold on
end
% disturbances
figure
disturbances = [];
values = [];
for it=1:length(controller)
    disturbances = controller(it).disturbances;
    values = controller(it).fitnessPerDisturbance;
    plot(disturbances,values,'color',rand(1,3))
    hold on
end
% controlLoop
figure
loopTimes = [];
values = [];
for it=1:length(controller)
    loopTimes = controller(it).controlLoops;
    values = controller(it).fitnessPerControlLoopTime;
    plot(loopTimes,values,'color',rand(1,3))
    hold on
end
% payloads
figure
payloads = [];
values = [];
for it=1:length(controller)
    payloads = controller(it).payloads(:,1);
    values = controller(it).fitnessPerPayload;
    plot(payloads,values,'color',rand(1,3))
    hold on
end