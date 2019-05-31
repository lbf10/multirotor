function evaluatePassiveController(filename)
data = load(filename);
[pathname,name,extension] = fileparts(filename);
foldername = ['Evaluation_',name]
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
controllerConfig = strsplit(name,'_');
attitudeController = controllerConfig{1}
controlAllocator = controllerConfig{2}
attitudeReference = controllerConfig{3}

disp([attitudeController,' - Starting Controller evaluation ',datestr(now)])
positions = [[0.282843 0.282843 0.05]',[-0.282843 0.282843 0.05]',[-0.282843 -0.282843 0.05]',[0.282843 -0.282843 0.05]',[0.282843 0.282843 -0.05]',[-0.282843 0.282843 -0.05]',[-0.282843 -0.282843 -0.05]',[0.282843 -0.282843 -0.05]'];
payloadRadius = 0.3*mean(sqrt(sum(positions.^2)));

samples = load('../monteCarloSamples/monteCarlo_samples.mat');
samplesFields = fields(samples);

for jt = 1:numel(samplesFields)
    if ~strcmp(samplesFields{jt},'columnNames') && ~strcmp(samplesFields{jt},'N4')
        disp([samplesFields{jt},' - Starting samples matrix ',datestr(now)])
        options = samples.(samplesFields{jt});
        options(1,end+1) = {0};
        options(1,end+1) = {0};
        numberOfOptions = length(options);
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
            multirotor.setTimeStep(0.002);
            multirotor.setControlTimeStep(option{6});
            multirotor.configFDD(0.99,0.2);
            multirotor.setSimEffects('motor dynamics on','solver ode45')
            multirotor.setControlDelay(option{7});
            multirotor = paramsToMultirotor(attitudeController, controlAllocator, attitudeReference, multirotor, bestIndividual);
            

            endTime = option{2};
            [waypoints, time] = geronoToWaypoints(7, 4, 4, endTime, endTime/8, 'goto',2*pi);
            multirotor.setTrajectory('waypoints',waypoints,time);
            position = [0,0,-payloadRadius];
            mass = option{1}*multirotor.mass();
            multirotor.setPayload(position,mass,eye(3)*2*mass*payloadRadius*payloadRadius/5);
            crossTime1 = num2str(endTime/4);
            crossTime2 = num2str(3*endTime/4);
            disturbance = option{3}*multirotor.mass();
            if disturbance ~= 0
                multirotor.setLinearDisturbance(['@(t) [0;1;0]*',num2str(disturbance),'*(exp(-(t-',crossTime1,')^2/(0.5))+exp(-(t-',crossTime2,')^2/(0.5)))']);
            end
            failure = option{5};
            nFails = length(failure);
            step = endTime/2/(1+nFails);
            multirotor.addCommand(failure,endTime/2+step:step:endTime-0.000001);
            try
                multirotor.run('visualizeGraph',false,'visualizeProgress',false,'metricPrecision',0.15,'angularPrecision',5);
                metrics = multirotor.metrics();          
                option{8} = 1*(1-metrics.simulationSuccess)+metrics.RMSPositionError+real(metrics.RMSAngularError)+metrics.RMSPower/500;
            catch
                option{8} = 10;
            end
            option{9} = multirotor.metrics();
            options(it,:) = option;
%            disp(['Finished calculation ',num2str(it)])
        end
        samples.(samplesFields{jt}) = options;
        disp([samplesFields{jt},' - Finished samples matrix ',datestr(now)])
        save([pathname,'/',foldername,'/evaluationResult.mat'],'samples');
    end
end
end