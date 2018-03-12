function fitness = controlFitness(attitudeController, controlAllocator, attitudeReference, x)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    
    positions = [[0.34374 0.34245 0.0143]',[-0.341 0.34213 0.0143]',[-0.34068 -0.34262 0.0143]',[0.34407 -0.34229 0.0143]',[0.33898 0.33769 0.0913]',[-0.33624 0.33736 0.0913]',[-0.33591 -0.33785 0.0913]',[0.3393 -0.33753 0.0913]'];
    mass = 6.015;
    payloadRadius = 0.3*mean(sqrt(sum(positions.^2)));
    endTimes = [30];
    yawGoTos = [0,2*pi];
    payloads = [0, 0, 0, 0
               0.5*mass, 0, 0, 0
               0.5*mass, -payloadRadius/(2*sqrt(2)), -payloadRadius/(2*sqrt(2)), -payloadRadius
               1*mass, 0, 0, 0
               1*mass, -payloadRadius/(2*sqrt(2)), -payloadRadius/(2*sqrt(2)), -payloadRadius];
    disturbances = [0,5];
    failures = {{''}
                {'setRotorStatus(1,''motor loss'',0)'}
                {'setRotorStatus(1,''motor loss'',0)','setRotorStatus(2,''motor loss'',0)'}
                {'setRotorStatus(1,''motor loss'',0)','setRotorStatus(6,''motor loss'',0)'}
                {'setRotorStatus(1,''motor loss'',0)','setRotorStatus(2,''motor loss'',0)','setRotorStatus(3,''motor loss'',0)'}
                {'setRotorStatus(1,''motor loss'',0)','setRotorStatus(2,''motor loss'',0)','setRotorStatus(3,''motor loss'',0)','setRotorStatus(4,''motor loss'',0)'}};
      
    numberOfOptions = length(endTimes)*length(yawGoTos)*size(payloads,1)*length(disturbances)*length(failures);
    options = zeros(numberOfOptions,5);
    it = 1;
    for it1 = 1:length(endTimes)
        for it2 = 1:length(yawGoTos)
            for it3 = 1:size(payloads,1)
                for it4 = 1:length(disturbances)
                    for it5 = 1:length(failures)
                        options(it,:) = [it1,it2,it3,it4,it5];
                        it = it + 1;
                    end
                end
            end
        end
    end
    fitness = zeros(numberOfOptions,7);
    
    parfor it = 1:numberOfOptions
        % Creates simulation class
        multirotor = multicontrol(8);
        multirotor.supressVerbose()
        % Define rotor positions
        positions = [[0.34374 0.34245 0.0143]',[-0.341 0.34213 0.0143]',[-0.34068 -0.34262 0.0143]',[0.34407 -0.34229 0.0143]',[0.33898 0.33769 0.0913]',[-0.33624 0.33736 0.0913]',[-0.33591 -0.33785 0.0913]',[0.3393 -0.33753 0.0913]'];
        multirotor.setRotorPosition(1:8,positions);
        % Define rotor orientations
        orientations = [[-0.061628417 -0.061628417 0.996194698]',[0.061628417 -0.061628417 0.996194698]',[0.061628417 0.061628417 0.996194698]',[-0.061628417 0.061628417 0.996194698]',[-0.061628417 -0.061628417 0.996194698]',[0.061628417 -0.061628417 0.996194698]',[0.061628417 0.061628417 0.996194698]',[-0.061628417 0.061628417 0.996194698]'];
        multirotor.setRotorOrientation(1:8,orientations);
        % Define aircraft's inertia
        multirotor.setMass(6.015);
        inertia =   [0.3143978800	0.0000861200	-0.0014397600
                    0.0000861200	0.3122127800	0.0002368800
                    -0.0014397600	0.0002368800	0.5557912400];
        multirotor.setInertia(inertia);
        % Define aircraft's drag coeff
        friction = [0.25	0	0
                    0	0.25	0
                    0	0	0.25];
        multirotor.setFriction(friction);
        % Define lift and drag coefficients
        speed = [404.3449657
                416.5751859
                435.2676622
                462.5052705
                472.6526147
                491.345091
                501.4924353
                520.1849116
                530.3322559
                549.0247321
                567.7172084
                586.4096847
                748.2865294];
        liftCoeff = [0.00008877247161370610
                    0.00009663400821486720
                    0.00010197039400480800
                    0.00010177480503994200
                    0.00010886498777293000
                    0.00011048831185009000
                    0.00011230119869840700
                    0.00010908666646728400
                    0.00011227432775784800
                    0.00010996476733082600
                    0.00010862374599149600
                    0.00010409054272222600
                    0.00006567742093581670];
        dragCoeff = [0.00000100839872772950
                    0.00000115158401406177
                    0.00000131849846466781
                    0.00000140132963964922
                    0.00000156543968817590
                    0.00000165553807692624
                    0.00000178787094426600
                    0.00000184631980295481
                    0.00000195397512083756
                    0.00000198893164777812
                    0.00000201512348657737
                    0.00000203398711313428
                    0.00000136514255905061];
        multirotor.setRotorLiftCoeff(1:8,[speed, liftCoeff],'poly2');
        multirotor.setRotorDragCoeff(1:8,[speed, dragCoeff],'poly2');
        % Define rotor inertia
        multirotor.setRotorInertia(1:8,0.00047935*ones(1,8));
        % Sets rotors rotation direction for control allocation
        rotationDirection = [1 -1 1 -1 -1 1 -1 1]';
        multirotor.setRotorDirection(1:8,rotationDirection);
        multirotor.setRotorMaxSpeed(1:8,750*ones(1,8));
        multirotor.setRotorMinSpeed(1:8,328*ones(1,8));
        multirotor = paramsToMultirotor(attitudeController, controlAllocator, attitudeReference, multirotor, x);
        option = options(it,:);
        
        endTime = endTimes(option(1));
        [waypoints, time] = geronoToWaypoints(7, 4, 4, endTime, endTime/8, 'goto',yawGoTos(option(2)));
        multirotor.setTrajectory('waypoints',waypoints,time);
        position = payloads(option(3),2:4);
        mass = payloads(option(3),1);
        multirotor.setPayload(position,mass,eye(3)*2*mass*payloadRadius*payloadRadius/5);
        crossTime1 = num2str(endTime/4);
        crossTime2 = num2str(3*endTime/4);
        if disturbances(option(4)) ~= 0
            multirotor.setLinearDisturbance(['@(t) [0;1;0]*',num2str(disturbances(option(4))),'*(exp(-(t-',crossTime1,')^2/(0.5))+exp(-(t-',crossTime2,')^2/(0.5)))']);
        end
        nFails = length(failures{option(5)});
        step = endTime/4/(1+nFails);
        multirotor.addCommand(failures{option(5)},endTime/8+step:step:3*endTime/8-0.000001);
        
        try
            multirotor.run('visualizeGraph',false,'visualizeProgress',false,'metricPrecision',0.15,'angularPrecision',5);
            metrics = multirotor.metrics();
            fitness(it,:) = [metrics.RMSPositionError,metrics.maxPositionError, real(metrics.RMSAngularError), real(metrics.maxAngularError), metrics.energy, metrics.maxPower, metrics.missionSuccess];
        catch
            fitness(it,:) = [endTime*5,endTime*10, pi/2, pi, 1e10, 1e10, 0];
        end
        %disp(it)
    end    
    fitness = sum(fitness);
    fitness(end) = 1-fitness(end)/numberOfOptions;
    disp('Fitness Calculated')
end

