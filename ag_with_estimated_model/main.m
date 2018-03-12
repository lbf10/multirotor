%% Script to train control parameters using GA
% Author: Leonardo B. Farçoni
% Creation: 06/03/2018
matlabrc
addpath(genpath('../multiControl_v2.0/'))
warning('on','all')

%% Algorithms to train
attitudeController = 'PID';
controlAllocator = 'PI Passive';
attitudeReference = 'PI Passive';

%% Select nvars according to algorithms
initialPopulation = [54 54 54 3 3 3 15 15 15 1 1 1];
lb = zeros(1,12);
ub = 1e5*ones(1,12);
    switch attitudeController
        case 'PID'
            initialPopulation = [initialPopulation [250 250 250 100 100 100 22 22 22]];
            lb = [lb,zeros(1,9)];
            ub = [ub,1e5*ones(1,9)];
            nvars = 21;
        case 'RLQ-R Passive'
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
            c = 2*[1,1,0.001];
            alpha = 0.1*[1,1,0.001];
            lambda = 0.1*[0.1,0.1,0.001];
            initialPopulation = [initialPopulation,c,lambda,alpha];
            lb = [lb,-inf(1,9)];
            ub = [ub,inf(1,9)];
            nvars = 21;
        case 'SOSMC Passive with PIDD'
            c = 0.5*[1,1,1,2,2,2];
            alpha = 0.2*[0.01,0.01,1,2,2,2];
            lambda = 0.01*[1,1,1,2,2,2];
            initialPopulation = [initialPopulation,c,lambda,alpha];
            lb = [lb,-inf(1,18)];
            ub = [ub,inf(1,18)];
            nvars = 30;
        case 'SOSMC Passive Direct'
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
            c = 2*[1,1,0.001];
            alpha = 0.1*[1,1,0.001];
            lambda = 0.1*[0.1,0.1,0.001];
            initialPopulation = [initialPopulation,c,lambda,alpha];
            lb = [lb,-inf(1,9)];
            ub = [ub,inf(1,9)];
            nvars = 21;
        case 'SOSMC Active with PIDD'
            c = 0.5*[1,1,1,2,2,2];
            alpha = 0.2*[0.01,0.01,1,2,2,2];
            lambda = 0.01*[1,1,1,2,2,2];
            initialPopulation = [initialPopulation,c,lambda,alpha];
            lb = [lb,-inf(1,18)];
            ub = [ub,inf(1,18)];
            nvars = 30;
        case 'SOSMC Active Direct'
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
            Am = -[1,1,1,1,1,1];
            Q = 5*[1,1,1,1,1,1];
            gamma1 = [1,1,1,1,1,1,1,1]*10000000;
            gamma2 = [1,1,1,1,1,1,1,1]*10000000;
            gamma3 = [1,1,1,1,1,1,1,1]*10000000;
            gamma4 = [1,1,1,1,1,1,1,1]*10000000;
            initialPopulation = [initialPopulation,Am,Q,gamma1,gamma2,gamma3,gamma4];
            lb = [lb,-inf(1,6),zeros(1,38)];
            ub = [ub,zeros(1,6),inf(1,38)];
            nvars = 56;
    end
    switch controlAllocator
        case 'Adaptive'
            caGain = -2e12*[1,1,1,1,1,1];
            initialPopulation = [initialPopulation,caGain];
            lb = [lb,-inf(1,6)];
            ub = [ub,zeros(1,6)];
            nvars = nvars + 6;
    end
fitnessfcn = @(x) controlFitness(attitudeController, controlAllocator, attitudeReference, x);
%% Start GA
poolobj = parpool(70);
options = gaoptimset('UseParallel',false,'PopulationSize',200,'Generations',100,'Display','diagnose','InitialPopulation',initialPopulation);
[bestIndividual,bestFitness, EXITFLAG,OUTPUT,POPULATION,SCORES] = gamultiobj(fitnessfcn,nvars,[],[],[],[],lb,ub,[],options);
delete(poolobj)
filename = [attitudeController,'_',controlAllocator,'_',attitudeReference,'_',datestr(now),'.mat'];
finishDate = datestr(now);
save(filename,'attitudeController','controlAllocator','attitudeReference','bestIndividual','bestFitness','EXITFLAG','OUTPUT','POPULATION','SCORES','finishDate');