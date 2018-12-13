function fitness = controlFitness(attitudeController, controlAllocator, attitudeReference, x)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    
    disp([attitudeController,' - Starting Fitness Calculation ',datestr(now)])
%    x
    positions = [[0.282843 0.282843 0.05]',[-0.282843 0.282843 0.05]',[-0.282843 -0.282843 0.05]',[0.282843 -0.282843 0.05]',[0.282843 0.282843 -0.05]',[-0.282843 0.282843 -0.05]',[-0.282843 -0.282843 -0.05]',[0.282843 -0.282843 -0.05]'];
    payloadRadius = 0.3*mean(sqrt(sum(positions.^2)));
    endTimes = [15];
    yawGoTos = [2*pi];%[0,2*pi];
    payloads = [0, 0, 0, 0 	       	
		%0.5*mass, 0, 0, 0 	
		%0.5*mass, -payloadRadius/(2*sqrt(2)), -payloadRadius/(2*sqrt$ %1*mass, 0, 0, 0 disturbances = [5];
               	0.4, 0, 0, -payloadRadius]; 
    disturbances = [5];
    failures = {%{''}
                %{'setRotorStatus(1,''motor loss'',0.75)'}
                %{'setRotorStatus(1,''motor loss'',0.001)'}
                %{'setRotorStatus(1,''motor loss'',0.001)','setRotorStatus(2,''motor loss'',0.001)'}
                %{'setRotorStatus(1,''motor loss'',0.001)','setRotorStatus(6,''motor loss'',0.4)'}
                %{'setRotorStatus(1,''motor loss'',0.001)','setRotorStatus(2,''motor loss'',0.001)','setRotorStatus(3,''motor loss'',0.001)'}
                {'setRotorStatus(1,''motor loss'',0.001)','setRotorStatus(2,''motor loss'',0.001)','setRotorStatus(3,''motor loss'',0.001)','setRotorStatus(4,''motor loss'',0.001)'}};
      
    numberOfOptions = size(x,1)*length(endTimes)*length(yawGoTos)*size(payloads,1)*length(disturbances)*length(failures);
    options = cell(numberOfOptions,6);
    it = 1;
    for it6 = 1:size(x,1)
        for it1 = 1:length(endTimes)
            for it2 = 1:length(yawGoTos)
                for it3 = 1:size(payloads,1)
                    for it4 = 1:length(disturbances)
                        for it5 = 1:length(failures)
                                options(it,:) = {endTimes(it1),yawGoTos(it2),payloads(it3,:),disturbances(it4),failures{it5},x(it6,:)};
                                it = it + 1;
                        end
                    end
                end
            end
        end
    end
    fitness = zeros(numberOfOptions,1);
    
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
        multirotor = paramsToMultirotor(attitudeController, controlAllocator, attitudeReference, multirotor, option{6});
        
        
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
        multirotor.addCommand(failure,endTime/2+step:step:endTime-0.000001);
        
        try
            multirotor.run('visualizeGraph',false,'visualizeProgress',false,'metricPrecision',0.15,'angularPrecision',5);
            metrics = multirotor.metrics();          
            fitness(it) = 1*(1-metrics.simulationSuccess)+metrics.RMSPositionError+real(metrics.RMSAngularError)+metrics.RMSPower/500;
        catch
            fitness(it) = 10;
        end
%         multirotor.plotSim()
%         multirotor.metrics()
%         pause
        
        %disp(it)
    end    
    fitness = sum(reshape(fitness,[numberOfOptions/size(x,1),size(x,1)]))';
    disp(['Fitness Calculated ',datestr(now)])
end

