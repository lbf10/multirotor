clc
addpath('../multiControl/')
addpath('../multiControl/utils')
warning('off','all')
%% Inicialização
% Abre arquivo
[filename, pathname] = uigetfile('*.mat', 'Pick a MATLAB data file');
data = load([pathname,filename]);
foldername = strsplit(filename,'.mat');
foldername = ['Evaluation_',foldername{1}];
foldername = strrep(foldername,' ','_');
mkdir(pathname,foldername);

% Verifica se é arquivo finalizado de simulação ou não e retira melhor
% indivíduo da otimização
if any(strcmp(fieldnames(data),'bestIndividual'))
    bestIndividual = data.bestIndividual;
    bestScore = data.bestFitness;
else
    scores = data.state(end).Score;
    population = data.state(end).Population;
    [bestScore,bestIndex] = min(scores);
    bestIndividual = population(bestIndex,:);
end

%% Simulações
controllerConfig = strsplit(filename,'_');
attitudeController = controllerConfig{1};
controlAllocator = controllerConfig{2};
attitudeReference = controllerConfig{3};

disp([attitudeController,' - Starting Controller evaluation',datestr(now)])
positions = [[0.282843 0.282843 0.05]',[-0.282843 0.282843 0.05]',[-0.282843 -0.282843 0.05]',[0.282843 -0.282843 0.05]',[0.282843 0.282843 -0.05]',[-0.282843 0.282843 -0.05]',[-0.282843 -0.282843 -0.05]',[0.282843 -0.282843 -0.05]'];
payloadRadius = 0.3*mean(sqrt(sum(positions.^2)));
endTimes = [30,25,20,15,10];
yawGoTos = [2*pi];
mass = 1.64;
payloads = [0,         0, 0, 0
            0.25*mass, 0, 0, -payloadRadius
            0.50*mass, 0, 0, -payloadRadius
            0.75*mass, 0, 0, -payloadRadius
            mass,      0, 0, -payloadRadius];
disturbances = mass*[0.0, 1.5, 3.0, 4.5, 6.0];
failures = {{''}
            {'setRotorStatus(1,''motor loss'',0.70)'}
            {'setRotorStatus(1,''motor loss'',0.50)'}
            {'setRotorStatus(1,''motor loss'',0.001)'}
            {'setRotorStatus(8,''motor loss'',0.70)'}
            {'setRotorStatus(8,''motor loss'',0.50)'}
            {'setRotorStatus(8,''motor loss'',0.001)'}
            {'setRotorStatus(1,''motor loss'',0.001)','setRotorStatus(2,''motor loss'',0.7)'}
            {'setRotorStatus(1,''motor loss'',0.001)','setRotorStatus(2,''motor loss'',0.001)'}
            {'setRotorStatus(1,''motor loss'',0.001)','setRotorStatus(2,''motor loss'',0.001)','setRotorStatus(3,''motor loss'',0.70)'}
            {'setRotorStatus(1,''motor loss'',0.001)','setRotorStatus(2,''motor loss'',0.001)','setRotorStatus(3,''motor loss'',0.001)'}
            {'setRotorStatus(1,''motor loss'',0.001)','setRotorStatus(2,''motor loss'',0.001)','setRotorStatus(3,''motor loss'',0.001)','setRotorStatus(4,''motor loss'',0.7)'}
            {'setRotorStatus(1,''motor loss'',0.001)','setRotorStatus(2,''motor loss'',0.001)','setRotorStatus(3,''motor loss'',0.001)','setRotorStatus(4,''motor loss'',0.001)'}
            {'setRotorStatus(1,''motor loss'',0.001)','setRotorStatus(2,''motor loss'',0.001)','setRotorStatus(3,''motor loss'',0.001)','setRotorStatus(4,''motor loss'',0.001)','setRotorStatus(8,''motor loss'',0.9)'}
            {'setRotorStatus(1,''motor loss'',0.001)','setRotorStatus(2,''motor loss'',0.001)','setRotorStatus(3,''motor loss'',0.001)','setRotorStatus(4,''motor loss'',0.001)','setRotorStatus(8,''motor loss'',0.8)'}
            {'setRotorStatus(1,''motor loss'',0.001)','setRotorStatus(2,''motor loss'',0.001)','setRotorStatus(3,''motor loss'',0.001)','setRotorStatus(4,''motor loss'',0.001)','setRotorStatus(8,''motor loss'',0.7)'}
            {'setRotorStatus(1,''motor loss'',0.001)','setRotorStatus(6,''motor loss'',0.7)'}
            {'setRotorStatus(1,''motor loss'',0.001)','setRotorStatus(6,''motor loss'',0.001)'}
            {'setRotorStatus(1,''motor loss'',0.001)','setRotorStatus(6,''motor loss'',0.001)','setRotorStatus(7,''motor loss'',0.70)'}
            {'setRotorStatus(1,''motor loss'',0.001)','setRotorStatus(6,''motor loss'',0.001)','setRotorStatus(7,''motor loss'',0.001)'}
            {'setRotorStatus(1,''motor loss'',0.001)','setRotorStatus(2,''motor loss'',0.001)','setRotorStatus(7,''motor loss'',0.70)'}
            {'setRotorStatus(1,''motor loss'',0.001)','setRotorStatus(2,''motor loss'',0.001)','setRotorStatus(7,''motor loss'',0.001)'}
            {'setRotorStatus(1,''motor loss'',0.001)','setRotorStatus(2,''motor loss'',0.001)','setRotorStatus(7,''motor loss'',0.001)','setRotorStatus(8,''motor loss'',0.7)'}
            {'setRotorStatus(1,''motor loss'',0.001)','setRotorStatus(2,''motor loss'',0.001)','setRotorStatus(7,''motor loss'',0.001)','setRotorStatus(8,''motor loss'',0.001)'}
            {'setRotorStatus(1,''motor loss'',0.001)','setRotorStatus(7,''motor loss'',0.7)'}
            {'setRotorStatus(1,''motor loss'',0.001)','setRotorStatus(7,''motor loss'',0.001)'}
            {'setRotorStatus(1,''motor loss'',0.001)','setRotorStatus(3,''motor loss'',0.7)'}
            {'setRotorStatus(1,''motor loss'',0.001)','setRotorStatus(3,''motor loss'',0.001)'}};

numberOfOptions = length(endTimes)*length(yawGoTos)*size(payloads,1)*length(disturbances)*length(failures);
options = cell(numberOfOptions,7);
it = 1;
for it1 = 1:length(endTimes)
    for it2 = 1:length(yawGoTos)
        for it3 = 1:size(payloads,1)
            for it4 = 1:length(disturbances)
                for it5 = 1:length(failures)
                        options(it,1:5) = {endTimes(it1),yawGoTos(it2),payloads(it3,:),disturbances(it4),failures{it5}};
                        it = it + 1;
                end
            end
        end
    end
end

parpool('local',8);
parfor it = 1:numberOfOptions
    option = options(it,:);

    % Creates simulation class
    multirotor = multicontrol(8);
    multirotor.supressVerbose()
    warning('off','all')
    % Define rotor positions
    positions = [[0.282843 0.282843 0.05]',[-0.282843 0.282843 0.05]',[-0.282843 -0.282843 0.05]',[0.282843 -0.282843 0.05]',[0.282843 0.282843 -0.05]',[-0.282843 0.282843 -0.05]',[-0.282843 -0.282843 -0.05]',[0.282843 -0.282843 -0.05]'];
    multirotor.setRotorPosition(1:8,positions);
    % Define rotor orientations
    orientations = [[0 0 1]',[0 0 1]',[0 0 1]',[0 0 1]',[0 0 1]',[0 0 1]',[0 0 1]',[0 0 1]'];
    multirotor.setRotorOrientation(1:8,orientations);
    % Define aircraft's inertia
    multirotor.setMass(1.64);
    inertia = diag([44e-3,44e-3,88e-3]);
    multirotor.setInertia(inertia);
    % Define aircraft's drag coeff
    friction = [0.25	0	0
                0	0.25	0
                0	0	0.25];
    multirotor.setFriction(friction);
    % Define lift and drag coefficients
    multirotor.setRotorLiftCoeff(1:8,ones(1,8)*10e-6);
    multirotor.setRotorDragCoeff(1:8,ones(1,8)*0.3e-6);
    % Define rotor inertia
    multirotor.setRotorInertia(1:8,90e-6*ones(1,8));
    % Sets rotors rotation direction for control allocation
    rotationDirection = [1 -1 1 -1 -1 1 -1 1]';
    multirotor.setRotorDirection(1:8,rotationDirection);
    multirotor.setRotorMaxSpeed(1:8,1500*ones(1,8));
    multirotor.setRotorMinSpeed(1:8,0*ones(1,8));
    multirotor.setInitialRotorSpeeds(328*rotationDirection);
    multirotor.setInitialInput(9.47*rotationDirection);
    multirotor.setInitialVelocity([0;0;0]);
    multirotor.setInitialPosition([0;0;0]);
    multirotor.setInitialAngularVelocity([0;0;0]);
    multirotor.setRotorOperatingPoint(1:8,352*[1 1 1 1 1 1 1 1]);
    multirotor.configControlAllocator('Passive NMAC',1,0);
    multirotor.configControlAllocator('Active NMAC',1,0);
    multirotor.setTimeStep(0.005);
    multirotor.setControlTimeStep(0.05);
    multirotor.configFDD(0.9,0.5);
    multirotor.setSimEffects('motor dynamics off','solver euler')
    multirotor.setControlDelay(0.20);
    multirotor = paramsToMultirotor(attitudeController, controlAllocator, attitudeReference, multirotor, bestIndividual);


    endTime = option{1};
    [waypoints, time] = geronoToWaypoints(7, 4, 4, endTime, endTime/8, 'goto',option{2});
    multirotor.setTrajectory('waypoints',waypoints,time);
    payload = option{3};
    position = payload(2:4);
    mass = payload(1);
    multirotor.setPayload(position,mass,eye(3)*2*mass*payloadRadius*payloadRadius/5);
    crossTime1 = num2str(endTime/4);
    crossTime2 = num2str(3*endTime/4);
    disturbance = option{4};
    if disturbance ~= 0
        multirotor.setLinearDisturbance(['@(t) [0;1;0]*',num2str(disturbance),'*(exp(-(t-',crossTime1,')^2/(0.5))+exp(-(t-',crossTime2,')^2/(0.5)))']);
    end
    failure = option{5};
    nFails = length(failure);
    step = endTime/2/(1+nFails);
    multirotor.addCommand(failure,step:step:endTime/2-0.000001);

    try
        multirotor.run('visualizeGraph',false,'visualizeProgress',false,'metricPrecision',0.15,'angularPrecision',5);
        metrics = multirotor.metrics();          
        option{6} = 1*(1-metrics.simulationSuccess)+metrics.RMSPositionError+real(metrics.RMSAngularError)+metrics.RMSPower/500;
    catch
        option{6} = inf;
    end
    option{7} = multirotor.metrics();
    options(it,:) = option;
    multirotor.save('all',[pathname,foldername,'/multirotorSimulation_',num2str(it),'.mat']);
%         multirotor.plotSim()
%         multirotor.metrics()
%         pause
end    
save([pathname,foldername,'/evaluationResult.mat'],'options');
disp(['Evaluation finished.',datestr(now)])
