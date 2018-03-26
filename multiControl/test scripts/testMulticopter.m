addpath(genpath('../'))
warning('on','all')
 clear all
% close all
% clc
%% Configuration for +8 coaxial octorotor
% Creates simulation class
multirotor = multicopter(8);
% multirotor.supressVerbose()
% Define rotor positions
% positions = [[0.34374 0.34245 0.0143]',[-0.341 0.34213 0.0143]',[-0.34068 -0.34262 0.0143]',[0.34407 -0.34229 0.0143]',[0.33898 0.33769 0.0913]',[-0.33624 0.33736 0.0913]',[-0.33591 -0.33785 0.0913]',[0.3393 -0.33753 0.0913]'];
positions = [[0.34374 0.34374 0.0143]',[-0.34374 0.34374 0.0143]',[-0.34374 -0.34374 0.0143]',[0.34374 -0.34374 0.0143]',[0.34374 0.34374 0.0913]',[-0.34374 0.34374 0.0913]',[-0.34374 -0.34374 0.0913]',[0.34374 -0.34374 0.0913]'];
multirotor.setRotorPosition(1:8,positions);
% Define rotor orientations
%orientations = [[-0.061628417 -0.061628417 0.996194698]',[0.061628417 -0.061628417 0.996194698]',[0.061628417 0.061628417 0.996194698]',[-0.061628417 0.061628417 0.996194698]',[-0.061628417 -0.061628417 0.996194698]',[0.061628417 -0.061628417 0.996194698]',[0.061628417 0.061628417 0.996194698]',[-0.061628417 0.061628417 0.996194698]'];
orientations = [[0 0 1]',[0 0 1]',[0 0 1]',[0 0 1]',[0 0 1]',[0 0 1]',[0 0 1]',[0 0 1]'];
multirotor.setRotorOrientation(1:8,orientations);
% Define aircraft's inertia
multirotor.setMass(6.015);
mass = 6;
payloadRadius = 0.3*mean(sqrt(sum(positions.^2)));
multirotor.setPayload([0, 0, -payloadRadius/1.5],mass,eye(3)*2*mass*payloadRadius*payloadRadius/5);
% multirotor.setPayload([0, 0, 0],mass,eye(3)*2*mass*payloadRadius*payloadRadius/5);
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
speed = [0
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
        748.2865294
        1000];    
liftCoeff = [0
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
            0.00006567742093581670
            0];   
 dragCoeff = [0
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
            0.00000136514255905061
            0];
multirotor.setRotorLiftCoeff(1:8,[speed liftCoeff],'smoothingspline');
multirotor.setRotorDragCoeff(1:8,[speed dragCoeff],'smoothingspline');
% multirotor.setRotorLiftCoeff(1:8,mean(liftCoeff));
% multirotor.setRotorDragCoeff(1:8,mean(dragCoeff));
% Define rotor inertia
multirotor.setRotorInertia(1:8,0.00047935*ones(1,8));
% Sets rotors rotation direction for control allocation
rotationDirection = [1 -1 1 -1 -1 1 -1 1]';
multirotor.setRotorMaxSpeed(1:8,750*ones(1,8));
multirotor.setRotorMinSpeed(1:8,0*ones(1,8));
% multirotor.setInitialRotorSpeeds(270*rotationDirection);
% multirotor.setInitialInput(270*rotationDirection);
multirotor.setInitialVelocity([0;0;0]);
multirotor.setInitialPosition([0;0;0]);
multirotor.setInitialAngularVelocity([0;0;0]);
multirotor.setRotorRm(1:8,0.0975*ones(1,8));
multirotor.setRotorKt(1:8,0.02498*ones(1,8));
multirotor.setRotorKv(1:8,340*ones(1,8));
multirotor.setRotorMaxVoltage(1:8,22*ones(1,8));

multirotor.setSimEffects('solver ode45','motor dynamics on')
multirotor.setTimeStep(0.005)

inputs = 22*rotationDirection;
time = [0.01,30];
multirotor.run(inputs,time)
% multirotor.setRotorStatus(1,'stuck',0.05);
% time = [5.01,10];
% multirotor.run(inputs,time)
figure
plot3(multirotor.log().position(1,:),multirotor.log().position(2,:),multirotor.log().position(3,:))
title('3D Position')
daspect([1 1 1])
figure
plot(multirotor.log().position(3,:))