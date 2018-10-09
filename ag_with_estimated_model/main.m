%% Script to train control parameters using GA
% Author: Leonardo B. Far�oni
% Creation: 06/03/2018
matlabrc
addpath(genpath('../multiControl/'))
warning('off','all')

%% Algorithms to train
attitudeController = 'SOSMC Passive';
controlAllocator = 'Passive NMAC';
attitudeReference = 'Passive NMAC';

fullfilename = 0;

%% Select nvars according to algorithms
    switch attitudeController
        case 'PID'
            initialPopulation = [220 220 220 40 40 40 50 50 50 10 10 10
                                 90 90 90 10 10 10 40 40 40 2 2 2];
            lb = zeros(1,12);
            ub = [1000 1000 1000 1000 1000 1000 1000 1000 1000 100 100 100];
            initialPopulation = [initialPopulation [500 500 250 1000 1000 650 60 60 5; 130 130 50 200 200 200 14 14 2]];
            lb = [lb,zeros(1,9)];
            ub = [ub,1000*ones(1,9)];
            nvars = 21;
            initialPopulation = [initialPopulation,[0,0,0.5;0,0,0.5]];
            lb = [lb,0,0,0];
            ub = [ub,1,1,1];
            nvars = nvars + 3;
        case 'RLQ-R Passive'
            initialPopulation = [30 70 100 10 20 40 20 50 70 3 5 2];
            lb = zeros(1,12);
            ub = [1000 1000 1000 1000 1000 1000 1000 1000 1000 100 100 100];
            Q = [1e4, 1e4, 1e3,1e-1,1e-1,1e-1];
            P = Q;
            R = 70*[1,1,.1];
            Ef = 0.1*[2 2 1 0 0 0];
            Eg = 0.02*[1 1 1];
            H = [1 1 1 1 1 1];
            mu = 1e30;
            alpha = 1.5;
            initialPopulation = [initialPopulation,P,Q,R,Ef,Eg,H,mu,alpha];
            lb = [lb,zeros(1,30),100,1];
            ub = [ub,1e7*ones(1,15),1e3*ones(1,15),1e30,100];
            nvars = 44;
            initialPopulation = [initialPopulation,0,0,0.5];
            lb = [lb,0,0,0];
            ub = [ub,1,1,1];
            nvars = nvars + 3;
        case 'RLQ-R Passive Modified'
            initialPopulation = [70 70 60 2 2 40 25 25 30 3 3 3];
            lb = zeros(1,12);
            ub = [1000 1000 1000 1000 1000 1000 1000 1000 1000 100 100 100];
            Q = 500000*[1e1, 1e1, 1e1,1e1,1e1,1e1];
            P = Q;
            R = 0.00001*[1,1,1,1,1,1,1,1];
            Ef = 10*[2 2 1 0 0 0];
            Eg = 1000*[1 1 1 1 1 1 1 1];
            H = [1 1 1 1 1 1];
            mu = 1e20;
            alpha = 1.5;
            initialPopulation = [initialPopulation,P,Q,R,Ef,Eg,H,mu,alpha];
            lb = [lb,zeros(1,40),100,1];
            ub = [ub,1e9*ones(1,12),1e3*ones(1,8),1e5*ones(1,20),1e30,100];
            nvars = 54;
            initialPopulation = [initialPopulation,0,0,0.5];
            lb = [lb,0,0,0];
            ub = [ub,1,1,1];
            nvars = nvars + 3;
        case 'RLQ-R Passive Modified with PIDD'
            initialPopulation = [70 70 100 5 5 40 40 40 30 2 2 1];
            lb = zeros(1,12);
            ub = [1000 1000 1000 1000 1000 1000 1000 1000 1000 100 100 100];
            Q = 500000*[1e1, 1e1, 1e1, 1e1, 1e1, 1e1,1e1,1e1,1e1];
            P = Q;
            R = 0.00001*[1,1,1,1,1,1,1,1];
            H = [1 1 1 1 1 1 1 1 1];
            Ef = 10*[1 1 1 1 1 1 0 0 0];
            Eg = 40*[1 1 1 1 1 1 1 1];
            mu = 1e22;
            alpha = 1.5;
            initialPopulation = [initialPopulation,P,Q,R,Ef,Eg,H,mu,alpha];
            lb = [lb,zeros(1,52),100,1];
            ub = [ub,1e9*ones(1,18),1e3*ones(1,8),1e5*ones(1,26),1e30,100];
            nvars = 66;
            initialPopulation = [initialPopulation,0,0,0.5];
            lb = [lb,0,0,0];
            ub = [ub,1,1,1];
            nvars = nvars + 3;
        case 'RLQ-R Active'
            initialPopulation = [30 70 100 10 20 40 20 50 70 3 5 2];
            lb = zeros(1,12);
            ub = [1000 1000 1000 1000 1000 1000 1000 1000 1000 100 100 100];
            Q = [1e4, 1e4, 1e3,1e-1,1e-1,1e-1];
            P = Q;
            R = 70*[1,1,.1];
            Ef = 0.1*[2 2 1 0 0 0];
            Eg = 0.02*[1 1 1];
            H = [1 1 1 1 1 1];
            mu = 1e30;
            alpha = 1.5;
            initialPopulation = [initialPopulation,P,Q,R,Ef,Eg,H,mu,alpha];
            lb = [lb,zeros(1,31),1];
            ub = [ub,inf(1,32)];
            nvars = 44;
            initialPopulation = [initialPopulation,0,0,0.5];
            lb = [lb,0,0,0];
            ub = [ub,1,1,1];
            nvars = nvars + 3;
        case 'RLQ-R Active Modified'
            initialPopulation = [70 70 60 2 2 40 25 25 30 3 3 3];
            lb = zeros(1,12);
            ub = [1000 1000 1000 1000 1000 1000 1000 1000 1000 100 100 100];
            Q = 500000*[1e1, 1e1, 1e1,1e1,1e1,1e1];
            P = Q;
            R = 0.00001*[1,1,1,1,1,1,1,1];
            Ef = 10*[2 2 1 0 0 0];
            Eg = 1000*[1 1 1 1 1 1 1 1];
            H = [1 1 1 1 1 1];
            mu = 1e20;
            alpha = 1.5;
            initialPopulation = [initialPopulation,P,Q,R,Ef,Eg,H,mu,alpha];
            lb = [lb,zeros(1,41),1];
            ub = [ub,inf(1,42)];
            nvars = 54;
            initialPopulation = [initialPopulation,0,0,0.5];
            lb = [lb,0,0,0];
            ub = [ub,1,1,1];
            nvars = nvars + 3;
        case 'RLQ-R Active Modified with PIDD'
            initialPopulation = [70 70 60 2 2 4 25 25 30 3 3 3];
            lb = zeros(1,12);
            ub = [1000 1000 1000 1000 1000 1000 1000 1000 1000 100 100 100];
            Q = 500000*[1e1, 1e1, 1e1, 1e1, 1e1, 1e1,1e1,1e1,1e1];
            P = Q;
            R = 0.00001*[1,1,1,1,1,1,1,1];
            H = [1 1 1 1 1 1 1 1 1];
            Ef = 10*[1 1 1 1 1 1 0 0 0];
            Eg = 40*[1 1 1 1 1 1 1 1];
            mu = 1e22;
            alpha = 1.5;
            initialPopulation = [initialPopulation,P,Q,R,Ef,Eg,H,mu,alpha];
            lb = [lb,zeros(1,53),1];
            ub = [ub,inf(1,54)];
            nvars = 66;
            initialPopulation = [initialPopulation,0,0,0.5];
            lb = [lb,0,0,0];
            ub = [ub,1,1,1];
            nvars = nvars + 3;
        case 'SOSMC Passive'
            initialPopulation = [300 300 100 10 10 40 70 70 70 35 35 2];
            lb = zeros(1,12);
            ub = [1000 1000 1000 1000 1000 1000 1000 1000 1000 100 100 100];
            c = 3*[1,1,1];
            alpha =  2*[1,1,1];
            lambda = .1*[1,1,1];
            initialPopulation = [initialPopulation,c,lambda,alpha];
            lb = [lb,-100*ones(1,9)];
            ub = [ub,100*ones(1,9)];
            nvars = 21;
            initialPopulation = [initialPopulation,0,0,0.5];
            lb = [lb,0,0,0];
            ub = [ub,1,1,1];
            nvars = nvars + 3;
        case 'SOSMC Passive with PIDD'
            initialPopulation = [300 300 100 10 10 40 70 70 70 35 35 2];
            lb = zeros(1,12);
            ub = [1000 1000 1000 1000 1000 1000 1000 1000 1000 100 100 100];
            c = 3*[1,1,1,2,2,2];
            alpha = 2*[1,1,1,2,2,2];
            lambda = .1*[1,1,1,2,2,2];
            initialPopulation = [initialPopulation,c,lambda,alpha];
            lb = [lb,-100*ones(1,18)];
            ub = [ub,100*ones(1,18)];
            nvars = 30;
            initialPopulation = [initialPopulation,0,0,0.5];
            lb = [lb,0,0,0];
            ub = [ub,1,1,1];
            nvars = nvars + 3;
        case 'SOSMC Passive Direct'
            initialPopulation = [100 100 100 10 10 40 80 90 70 35 35 1];
            lb = zeros(1,12);
            ub = [1000 1000 1000 1000 1000 1000 1000 1000 1000 100 100 100];
            c =1.5*[1,1,4,2,2,2];
            alpha = 5500*...
                       [ 1 -1  0.1 -0.001 -0.01 1 ...
                         1  1 -0.1  0.001 -0.01 1 ...
                        -1  1  0.1  0.001  0.01 1 ...
                        -1 -1 -0.1 -0.001  0.01 1 ...
                         1 -1 -0.1 -0.001 -0.01 1 ...
                         1  1  0.1  0.001 -0.01 1 ...
                        -1  1 -0.1  0.001  0.01 1 ...
                        -1 -1  0.1 -0.001  0.01 1];
            lambda = 1000*...
                        [1 -1  0.1 -0.001 -0.001 1 ...
                         1  1 -0.1  0.001 -0.001 1 ...
                        -1  1  0.1  0.001  0.001 1 ...
                        -1 -1 -0.1 -0.001  0.001 1 ...
                         1 -1 -0.1 -0.001 -0.001 1 ...
                         1  1  0.1  0.001 -0.001 1 ...
                        -1  1 -0.1  0.001  0.001 1 ...
                        -1 -1  0.1 -0.001  0.001 1];
            initialPopulation = [initialPopulation,c,lambda,alpha];
            auxLB = [ 0 -1  0 -1 -1 0 ...
                                        0  0 -1  0 -1 0 ...
                                       -1  0  0  0  0 0 ...
                                       -1 -1 -1 -1  0 0 ...
                                        0 -1 -1 -1 -1 0 ...
                                        0  0  0  0 -1 0 ...
                                       -1  0 -1  0  0 0 ...
                                       -1 -1  0 -1  0 0];
            auxUB = [  1  0  1  0  0 1 ...
                                            1  1  0  1  0 1 ...
                                            0  1  1  1  1 1 ...
                                            0  0  0  0  1 1 ...
                                            1  0  0  0  0 1 ...
                                            1  1  1  1  0 1 ...
                                            0  1  0  1  1 1 ...
                                            0  0  1  0  1 1];
            lb = [lb,zeros(1,6),50000*auxLB,50000*auxLB];
            ub = [ub,100*ones(1,6),50000*auxUB,50000*auxUB];
            nvars = 114;
            initialPopulation = [initialPopulation,0,0,0.5];
            lb = [lb,0,0,0];
            ub = [ub,1,1,1];
            nvars = nvars + 3;
        case 'SOSMC Active'
            initialPopulation = [300 300 100 10 10 40 70 70 70 35 35 2];
            lb = zeros(1,12);
            ub = [1000 1000 1000 1000 1000 1000 1000 1000 1000 100 100 100];
            c = 3*[1,1,1];
            alpha =  2*[1,1,1];
            lambda = .1*[1,1,1];
            initialPopulation = [initialPopulation,c,lambda,alpha];
            lb = [lb,-inf(1,9)];
            ub = [ub,inf(1,9)];
            nvars = 21;
            initialPopulation = [initialPopulation,0,0,0.5];
            lb = [lb,0,0,0];
            ub = [ub,1,1,1];
            nvars = nvars + 3;
        case 'SOSMC Active with PIDD'
            initialPopulation = [300 300 100 10 10 40 70 70 70 35 35 2];
            lb = zeros(1,12);
            ub = [1000 1000 1000 1000 1000 1000 1000 1000 1000 100 100 100];           
            c = 3*[1,1,1,2,2,2];
            alpha = 2*[1,1,1,2,2,2];
            lambda = .1*[1,1,1,2,2,2];
            initialPopulation = [initialPopulation,c,lambda,alpha];
            lb = [lb,-inf(1,18)];
            ub = [ub,inf(1,18)];
            nvars = 30;
            initialPopulation = [initialPopulation,0,0,0.5];
            lb = [lb,0,0,0];
            ub = [ub,1,1,1];
            nvars = nvars + 3;
        case 'SOSMC Active Direct'
            initialPopulation = [100 100 100 10 10 40 80 90 70 35 35 1];
            lb = zeros(1,12);
            ub = [1000 1000 1000 1000 1000 1000 1000 1000 1000 100 100 100];
            c =1.5*[1,1,4,2,2,2];
            alpha = 5500*...
                       [ 1 -1  0.1 -0.001 -0.01 1 ...
                         1  1 -0.1  0.001 -0.01 1 ...
                        -1  1  0.1  0.001  0.01 1 ...
                        -1 -1 -0.1 -0.001  0.01 1 ...
                         1 -1 -0.1 -0.001 -0.01 1 ...
                         1  1  0.1  0.001 -0.01 1 ...
                        -1  1 -0.1  0.001  0.01 1 ...
                        -1 -1  0.1 -0.001  0.01 1];
            lambda = 1000*...
                        [1 -1  0.1 -0.001 -0.001 1 ...
                         1  1 -0.1  0.001 -0.001 1 ...
                        -1  1  0.1  0.001  0.001 1 ...
                        -1 -1 -0.1 -0.001  0.001 1 ...
                         1 -1 -0.1 -0.001 -0.001 1 ...
                         1  1  0.1  0.001 -0.001 1 ...
                        -1  1 -0.1  0.001  0.001 1 ...
                        -1 -1  0.1 -0.001  0.001 1];
            initialPopulation = [initialPopulation,c,lambda,alpha];
            lb = [lb,-inf(1,102)];
            ub = [ub,inf(1,102)];
            nvars = 111;
            initialPopulation = [initialPopulation,0,0,0.5];
            lb = [lb,0,0,0];
            ub = [ub,1,1,1];
            nvars = nvars + 3;
        case 'Adaptive'
            initialPopulation = [70 70 100 10 10 40 40 40 70 15 15 2];
            lb = zeros(1,12);
            ub = [1000 1000 1000 1000 1000 1000 1000 1000 1000 100 100 100];
            Am = -2*[1,1,1e-2];
            Q = 1*[.5,.5,.0005];
            gamma1 = [1,1,2]*.8;
            gamma2 = [1,1,2]*.2;
            gamma3 = [1,1,.1]*10;
            gamma4 = [1,1,1]*0;
            initialPopulation = [initialPopulation,Am,Q,gamma1,gamma2,gamma3,gamma4];
            lb = [lb,-inf(1,3),zeros(1,15)];
            ub = [ub,zeros(1,3),inf(1,15)];
            nvars = 30;
            initialPopulation = [initialPopulation,0,0,0.5];
            lb = [lb,0,0,0];
            ub = [ub,1,1,1];
            nvars = nvars + 3;
        case 'Adaptive with PIDD'
            initialPopulation = [70 70 100 10 10 40 40 40 70 15 15 2];
            lb = zeros(1,12);
            ub = [1000 1000 1000 1000 1000 1000 1000 1000 1000 100 100 100];
            Am = -[.1,.1,2,2,2,0.01];
            Q = [1,1,1,.5,.5,.0005];
            gamma1 = [1,1,1,1,1,1]*.0001;
            gamma2 = [1,1,1,1,1,1]*.0001;
            gamma3 = [1,1,1,1,1,1]*.001;
            gamma4 = [1,1,1,1,1,1]*.00001;
            initialPopulation = [initialPopulation,Am,Q,gamma1,gamma2,gamma3,gamma4];
            lb = [lb,-inf(1,6),zeros(1,30)];
            ub = [ub,zeros(1,6),inf(1,30)];
            nvars = 48;
            initialPopulation = [initialPopulation,0,0,0.5];
            lb = [lb,0,0,0];
            ub = [ub,1,1,1];
            nvars = nvars + 3;
        case 'Adaptive Direct'
            initialPopulation = [60 60 100 20 20 40 40 40 70 15 15 2];
            lb = zeros(1,12);
            ub = [1000 1000 1000 1000 1000 1000 1000 1000 1000 100 100 100];
            Am = -[.1,.1,2,0.001,0.001,0.001];
            Q = [1,1,1,0.005,0.005,.0005];
            gamma1 = ones(1,8)*700;
            gamma2 = ones(1,8)*700;
            gamma3 = ones(1,8)*2000;
            gamma4 = ones(1,8)*0.5;
            B0 = 5e3*[ -0.00001 -0.00001 4  1.5 -1.5  -3 ...
                         0.00001 -0.00001 4  1.5  1.5 3  ...
                         0.00001  0.00001 4 -1.5  1.5  -3  ...
                        -0.00001  0.00001 4 -1.5 -1.5 3 ...
                        -0.00001 -0.00001 4  1.5 -1.5 3 ...
                         0.00001 -0.00001 4  1.5  1.5  -3  ...
                         0.00001  0.00001 4 -1.5  1.5 3  ...
                        -0.00001  0.00001 4 -1.5 -1.5  -3];
            initialPopulation = [initialPopulation,Am,Q,gamma1,gamma2,gamma3,gamma4,B0];
            lb = [lb,-inf(1,6),zeros(1,38),-inf(1,48)];
            ub = [ub,zeros(1,6),inf(1,38),inf(1,48)];
            nvars = 104;
            initialPopulation = [initialPopulation,0,0,0.5];
            lb = [lb,0,0,0];
            ub = [ub,1,1,1];
            nvars = nvars + 3;
    end
    
    switch controlAllocator
        case 'Adaptive'
            caGain = -2e12*[1,1,1,1,1,1];
            initialPopulation = [initialPopulation,caGain];
            lb = [lb,-inf(1,6)];
            ub = [ub,zeros(1,6)];
            nvars = nvars + 6;
    end
%     initialPopulation = [initialPopulation;initialPopulation;initialPopulation;initialPopulation]

%% Check for restore file
if fullfilename ~= 0
    savedState = load(fullfilename,'state');
    %initialPopulationAux = savedState.state(end).Population;
    initialPopulation = savedState.state(end).Population;
    %fitnessValues = savedState.state(end).Score;
    %initialPopulation = initialPopulationAux(find(fitnessValues==min(fitnessValues)),:)
end
fitnessfcn = @(x) controlFitness(attitudeController, controlAllocator, attitudeReference, x);
%% Start GA
filename = ['Results/',attitudeController,'_',controlAllocator,'_',attitudeReference,'_',datestr(now),'_result.mat'];
iterFilename = ['Results/',attitudeController,'_',controlAllocator,'_',attitudeReference,'_',datestr(now),'_iterations.mat'];
outFunction = @(options,state,flag) saveIter(options,state,flag,iterFilename);
options = gaoptimset('PopulationSize',1500,'Generations',300,'Display','iter','InitialPopulation',initialPopulation,'OutputFcn',outFunction,'Vectorized','on','CrossoverFraction',0.5,'MutationFcn', {@mutationuniform, 0.05}, 'StallGenLimit',25);
poolobj = parpool(80);
addAttachedFiles(poolobj,{'controlFitness.m','saveIter.m','paramsToMultirotor.m','../multiControl/'})
[bestIndividual,bestFitness, EXITFLAG,OUTPUT,POPULATION,SCORES] = ga(fitnessfcn,nvars,[],[],[],[],lb,ub,[],options);
delete(poolobj)

finishDate = datestr(now);
save(filename,'attitudeController','controlAllocator','attitudeReference','bestIndividual','bestFitness','EXITFLAG','OUTPUT','POPULATION','SCORES','finishDate','options','initialPopulation');
