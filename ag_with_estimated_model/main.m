%% Script to train control parameters using GA
% Author: Leonardo B. Far�oni
% Creation: 06/03/2018
matlabrc
addpath(genpath('../multiControl/'))
warning('on','all')

%% Algorithms to train
attitudeController = 'PID';
controlAllocator = 'Passive NMAC';
attitudeReference = 'Passive NMAC';

%% Select nvars according to algorithms
    switch attitudeController
        case 'PID'
            initialPopulation = [220 220 220 40 40 40 50 50 50 10 10 10
                                 90 90 90 10 10 10 40 40 40 2 2 2];
            lb = zeros(1,12);
            ub = 1e6*ones(1,12);
            initialPopulation = [initialPopulation [500 500 250 1000 1000 650 60 60 5; 130 130 50 200 200 200 14 14 2]];
            lb = [lb,zeros(1,9)];
            ub = [ub,1e6*ones(1,9)];
            nvars = 21;
        case 'RLQ-R Passive'
            initialPopulation = [100 100 100 20 20 20 40 40 40 0.1 0.1 0.1];
            lb = zeros(1,12);
            ub = 1e6*ones(1,12);
            P = 9e5*ones(1,6);
            Q = 40000*[1e-1, 1e-1, 1e-1,1e1,1e1,1e1];
            R = 10000*[1 1 1];
            Ef = 0.2*[1 1 1 0 0 0];
            Eg = 0.2*[1 1 1];
            H = [1 1 1 1 1 1];
            mu = 1e30;
            alpha = 1.5;
            initialPopulation = [initialPopulation,P,Q,R,Ef,Eg,H,mu,alpha];
            lb = [lb,zeros(1,32)];
            ub = [ub,inf(1,32)];
            nvars = 44;
        case 'RLQ-R Passive Modified'
            initialPopulation = [100 100 100 20 20 20 40 40 40 0.1 0.1 0.1];
            lb = zeros(1,12);
            ub = 1e6*ones(1,12);
            P = 9e5*ones(1,6);
            Q = 50000*[1e-1, 1e-1, 1e-1,1e1,1e1,1e1];
            R = 0.000001*ones(1,8);
            Ef = 0.2*[1 1 1 0 0 0];
            Eg = 0.2*[1 1 1 1 1 1 1 1];
            H = [1 1 1 1 1 1];
            mu = 1e20;
            alpha = 1.5;
            initialPopulation = [initialPopulation,P,Q,R,Ef,Eg,H,mu,alpha];
            lb = [lb,zeros(1,42)];
            ub = [ub,inf(1,42)];
            nvars = 54;
        case 'RLQ-R Passive Modified with PIDD'
            initialPopulation = [100 100 100 20 20 20 40 40 40 0.1 0.1 0.1];
            lb = zeros(1,12);
            ub = 1e6*ones(1,12);
            P = 9e5*ones(1,9);
            Q = 50000*[1e1, 1e1, 1e1, 1e-1, 1e-1, 1e-1,1e1,1e1,1e1];
            R = 0.000001*ones(1,8);
            Ef = 0.2*[1 1 1 1 1 1 0 0 0];
            Eg = 0.2*[1 1 1 1 1 1 1 1];
            H = [1 1 1 1 1 1 1 1 1];
            mu = 1e20;
            alpha = 1.5;
            initialPopulation = [initialPopulation,P,Q,R,Ef,Eg,H,mu,alpha];
            lb = [lb,zeros(1,54)];
            ub = [ub,inf(1,54)];
            nvars = 66;
        case 'RLQ-R Active'
            initialPopulation = [100 100 100 20 20 20 40 40 40 0.1 0.1 0.1];
            lb = zeros(1,12);
            ub = 1e6*ones(1,12);
            P = 9e5*ones(1,6);
            Q = 40000*[1e-1, 1e-1, 1e-1,1e1,1e1,1e1];
            R = 10000*[1 1 1];
            Ef = 0.2*[1 1 1 0 0 0];
            Eg = 0.2*[1 1 1];
            H = [1 1 1 1 1 1];
            mu = 1e30;
            alpha = 1.5;
            initialPopulation = [initialPopulation,P,Q,R,Ef,Eg,H,mu,alpha];
            lb = [lb,zeros(1,32)];
            ub = [ub,inf(1,32)];
            nvars = 44;
        case 'RLQ-R Active Modified'
            initialPopulation = [100 100 100 20 20 20 40 40 40 0.1 0.1 0.1];
            lb = zeros(1,12);
            ub = 1e6*ones(1,12);
            P = 9e5*ones(1,6);
            Q = 50000*[1e-1, 1e-1, 1e-1,1e1,1e1,1e1];
            R = 0.000001*ones(1,8);
            Ef = 0.2*[1 1 1 0 0 0];
            Eg = 0.2*[1 1 1 1 1 1 1 1];
            H = [1 1 1 1 1 1];
            mu = 1e20;
            alpha = 1.5;
            initialPopulation = [initialPopulation,P,Q,R,Ef,Eg,H,mu,alpha];
            lb = [lb,zeros(1,42)];
            ub = [ub,inf(1,42)];
            nvars = 54;
        case 'RLQ-R Active Modified with PIDD'
            initialPopulation = [100 100 100 20 20 20 40 40 40 0.1 0.1 0.1];
            lb = zeros(1,12);
            ub = 1e6*ones(1,12);
            P = 9e5*ones(1,9);
            Q = 50000*[1e1, 1e1, 1e1, 1e-1, 1e-1, 1e-1,1e1,1e1,1e1];
            R = 0.000001*ones(1,8);
            Ef = 0.2*[1 1 1 1 1 1 0 0 0];
            Eg = 0.2*[1 1 1 1 1 1 1 1];
            H = [1 1 1 1 1 1 1 1 1];
            mu = 1e20;
            alpha = 1.5;
            initialPopulation = [initialPopulation,P,Q,R,Ef,Eg,H,mu,alpha];
            lb = [lb,zeros(1,54)];
            ub = [ub,inf(1,54)];
            nvars = 66;
        case 'SOSMC Passive'
            initialPopulation = [100 100 100 20 20 20 40 40 40 0.1 0.1 0.1];
            lb = zeros(1,12);
            ub = 1e6*ones(1,12);
            c = 2*[1,1,0.001];
            alpha = 0.1*[1,1,0.001];
            lambda = 0.1*[0.1,0.1,0.001];
            initialPopulation = [initialPopulation,c,lambda,alpha];
            lb = [lb,-inf(1,9)];
            ub = [ub,inf(1,9)];
            nvars = 21;
        case 'SOSMC Passive with PIDD'
            initialPopulation = [100 100 100 20 20 20 40 40 40 0.1 0.1 0.1];
            lb = zeros(1,12);
            ub = 1e6*ones(1,12);
            c = 0.5*[1,1,1,2,2,2];
            alpha = 0.2*[0.01,0.01,1,2,2,2];
            lambda = 0.01*[1,1,1,2,2,2];
            initialPopulation = [initialPopulation,c,lambda,alpha];
            lb = [lb,-inf(1,18)];
            ub = [ub,inf(1,18)];
            nvars = 30;
        case 'SOSMC Passive Direct'
            initialPopulation = [100 100 100 20 20 20 40 40 40 0.1 0.1 0.1];
            lb = zeros(1,12);
            ub = 1e6*ones(1,12);
            c =0*[1,1,1,0,0,1];
            alpha = -180000*...
                        [0  1  1 0 0 -1 ...
                         1  0 -1 0 0 -1 ...
                         0 -1  1 0 0 -1 ...
                        -1  0 -1 0 0 -1 ...
                         0  1 -1 0 0 -1 ...
                         1  0  1 0 0 -1 ...
                         0 -1 -1 0 0 -1 ...
                        -1  0  1 0 0 -1];
            lambda = -60000*...
                        [0  1  1 0 0 -1 ...
                         1  0 -1 0 0 -1 ...
                         0 -1  1 0 0 -1 ...
                        -1  0 -1 0 0 -1 ...
                         0  1 -1 0 0 -1 ...
                         1  0  1 0 0 -1 ...
                         0 -1 -1 0 0 -1 ...
                        -1  0  1 0 0 -1];
            initialPopulation = [initialPopulation,c,lambda,alpha];
            lb = [lb,-inf(1,102)];
            ub = [ub,inf(1,102)];
            nvars = 111;
        case 'SOSMC Active'
            initialPopulation = [100 100 100 20 20 20 40 40 40 0.1 0.1 0.1];
            lb = zeros(1,12);
            ub = 1e6*ones(1,12);
            c = 2*[1,1,0.001];
            alpha = 0.1*[1,1,0.001];
            lambda = 0.1*[0.1,0.1,0.001];
            initialPopulation = [initialPopulation,c,lambda,alpha];
            lb = [lb,-inf(1,9)];
            ub = [ub,inf(1,9)];
            nvars = 21;
        case 'SOSMC Active with PIDD'
            initialPopulation = [100 100 100 20 20 20 40 40 40 0.1 0.1 0.1];
            lb = zeros(1,12);
            ub = 1e6*ones(1,12);
            c = 0.5*[1,1,1,2,2,2];
            alpha = 0.2*[0.01,0.01,1,2,2,2];
            lambda = 0.01*[1,1,1,2,2,2];
            initialPopulation = [initialPopulation,c,lambda,alpha];
            lb = [lb,-inf(1,18)];
            ub = [ub,inf(1,18)];
            nvars = 30;
        case 'SOSMC Active Direct'
            initialPopulation = [100 100 100 20 20 20 40 40 40 0.1 0.1 0.1];
            lb = zeros(1,12);
            ub = 1e6*ones(1,12);
            c =0*[1,1,1,0,0,1];
            alpha = -180000*...
                        [0  1  1 0 0 -1 ...
                         1  0 -1 0 0 -1 ...
                         0 -1  1 0 0 -1 ...
                        -1  0 -1 0 0 -1 ...
                         0  1 -1 0 0 -1 ...
                         1  0  1 0 0 -1 ...
                         0 -1 -1 0 0 -1 ...
                        -1  0  1 0 0 -1];
            lambda = -60000*...
                        [0  1  1 0 0 -1 ...
                         1  0 -1 0 0 -1 ...
                         0 -1  1 0 0 -1 ...
                        -1  0 -1 0 0 -1 ...
                         0  1 -1 0 0 -1 ...
                         1  0  1 0 0 -1 ...
                         0 -1 -1 0 0 -1 ...
                        -1  0  1 0 0 -1];
            initialPopulation = [initialPopulation,c,lambda,alpha];
            lb = [lb,-inf(1,102)];
            ub = [ub,inf(1,102)];
            nvars = 111;
        case 'Adaptive'
            initialPopulation = [100 100 100 20 20 20 40 40 40 0.1 0.1 0.1];
            lb = zeros(1,12);
            ub = 1e6*ones(1,12);
            Am = -1*[1,1,1.5];
            Q = 150*[1 1 1];
            gamma1 = [1 1 1]*0.0000005;
            gamma2 = [1 1 1]*0.0000005;
            gamma3 = [1 1 1]*0.0000005;
            gamma4 = [1 1 1]*0.0000005;
            initialPopulation = [initialPopulation,Am,Q,gamma1,gamma2,gamma3,gamma4];
            lb = [lb,-inf(1,3),zeros(1,15)];
            ub = [ub,zeros(1,3),inf(1,15)];
            nvars = 30;
        case 'Adaptive with PIDD'
            initialPopulation = [100 100 100 20 20 20 40 40 40 0.1 0.1 0.1];
            lb = zeros(1,12);
            ub = 1e6*ones(1,12);
            Am = -1*[1,1,1,1,1,1];
            Q = 10*[1,1,1,1,1,1];
            gamma1 = [1,1,1,1,1,1]*0.0000005;
            gamma2 = [1,1,1,1,1,1]*0.0000005;
            gamma3 = [1,1,1,1,1,1]*0.0000005;
            gamma4 = [1,1,1,1,1,1]*0.0000005;
            initialPopulation = [initialPopulation,Am,Q,gamma1,gamma2,gamma3,gamma4];
            lb = [lb,-inf(1,6),zeros(1,30)];
            ub = [ub,zeros(1,6),inf(1,30)];
            nvars = 48;
        case 'Adaptive Direct'
            initialPopulation = [100 100 100 20 20 20 40 40 40 0.1 0.1 0.1];
            lb = zeros(1,12);
            ub = 1e6*ones(1,12);
            Am = -15*[.1,.1,.1,1,1,5];
            Q = 5e3*[1,1,1,40,40,40];
            gamma1 = [1,1,1,1,1,1,1,1]*100;
            gamma2 = [1,1,1,1,1,1,1,1]*1;
            gamma3 = [1,1,1,1,1,1,1,1]*10;
            gamma4 = [1,1,1,1,1,1,1,1]*0.05;
            B0 = 3e3*[  -0.00001 -0.00001 4  1.5 -1.5  -1 ...
                         0.00001 -0.00001 4  1.5  1.5 1 ...
                         0.00001  0.00001 4 -1.5  1.5  -1 ...  
                        -0.00001  0.00001 4 -1.5 -1.5 1 ... 
                        -0.00001 -0.00001 4  1.5 -1.5 1 ... 
                         0.00001 -0.00001 4  1.5  1.5  -1  ... 
                         0.00001  0.00001 4 -1.5  1.5 1 ...  
                        -0.00001  0.00001 4 -1.5 -1.5  -1];
            initialPopulation = [initialPopulation,Am,Q,gamma1,gamma2,gamma3,gamma4,B0];
            lb = [lb,-inf(1,6),zeros(1,38),-inf(1,48)];
            ub = [ub,zeros(1,6),inf(1,38),inf(1,48)];
            nvars = 104;
    end
    switch controlAllocator
        case 'Adaptive'
            caGain = -2e12*[1,1,1,1,1,1];
            initialPopulation = [initialPopulation,caGain];
            lb = [lb,-inf(1,6)];
            ub = [ub,zeros(1,6)];
            nvars = nvars + 6;
    end
    initialPopulation = [initialPopulation;initialPopulation;initialPopulation;initialPopulation];

%% Check for restore file
[baseName, folder] = uigetfile();
fullFileName = fullfile(folder, baseName);
if folder ~= 0
    savedState = load(fullFileName,'state');
    initialPopulation = savedState.state(end).Population;
end
fitnessfcn = @(x) controlFitness(attitudeController, controlAllocator, attitudeReference, x);
%% Start GA
filename = ['Results/',attitudeController,'_',controlAllocator,'_',attitudeReference,'_',datestr(now),'_result.mat'];
iterFilename = ['Results/',attitudeController,'_',controlAllocator,'_',attitudeReference,'_',datestr(now),'_iterations.mat'];
outFunction = @(options,state,flag) saveIter(options,state,flag,iterFilename);
options = gaoptimset('UseParallel',false,'PopulationSize',500,'Generations',200,'Display','iter','InitialPopulation',initialPopulation,'OutputFcn',outFunction);
poolobj = parpool(4);
addAttachedFiles(poolobj,{'controlFitness.m','saveIter.m','paramsToMultirotor.m','../multiControl/'})
[bestIndividual,bestFitness, EXITFLAG,OUTPUT,POPULATION,SCORES] = ga(fitnessfcn,nvars,[],[],[],[],lb,ub,[],options);
delete(poolobj)

finishDate = datestr(now);
save(filename,'attitudeController','controlAllocator','attitudeReference','bestIndividual','bestFitness','EXITFLAG','OUTPUT','POPULATION','SCORES','finishDate');
