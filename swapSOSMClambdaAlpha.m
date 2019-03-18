%% Script to convert the SOSMC bestindividuals to the new standard (swapped alphas and lambdas)
% Abre arquivo
[filename, pathname] = uigetfile('*.mat', 'Pick a SOSMC MATLAB data file')
data = load([pathname,filename]);

controllerConfig = strsplit(filename,'_');
attitudeController = controllerConfig{1};
switch attitudeController
    case 'SOSMC Passive'
        lambdaIndex = 16:18;
        alphaIndex = 19:21;
    case 'SOSMC Passive with PIDD'
        lambdaIndex = 19:24;
        alphaIndex = 25:30;
    case 'SOSMC Passive Direct'
        lambdaIndex = 19:66;
        alphaIndex = 67:114;
end

if any(strcmp(fieldnames(data),'bestIndividual'))
    bestIndividual = data.bestIndividual;
    aux1 = bestIndividual(lambdaIndex);
    aux2 = bestIndividual(alphaIndex);
    bestIndividual(alphaIndex) = aux1;
    bestIndividual(lambdaIndex) = aux2;
    data.bestIndividual = bestIndividual;
    POPULATION = data.POPULATION;
    aux1 = POPULATION(:,lambdaIndex);
    aux2 = POPULATION(:,alphaIndex);
    POPULATION(:,alphaIndex) = aux1;
    POPULATION(:,lambdaIndex) = aux2;
    data.POPULATION = POPULATION;
    save([pathname,filename], 'POPULATION', '-append')
    save([pathname,filename], 'bestIndividual', '-append')
else
    for it=1:size(data.state,1)
        POPULATION = data.state(it).Population;
        aux1 = POPULATION(:,lambdaIndex);
        aux2 = POPULATION(:,alphaIndex);
        POPULATION(:,alphaIndex) = aux1;
        POPULATION(:,lambdaIndex) = aux2;
        Population = POPULATION;
        data.state(it).Population = POPULATION;
    end
    state = data.state;
    save([pathname,filename], 'state', '-append')    
end
