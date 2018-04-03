% Octorotor simulation
% refer to "doc multicontrol" or "doc multicopter" for more information

addpath(genpath('../'))
warning('on','all')
clear all
clc
%% Configuration for +8 coaxial octorotor
% Creates simulation class
multirotor = multicontrol(8);
% multirotor.supressVerbose()
% Define rotor positions
positions = [[0.34374 0.34245 0.0143]',[-0.341 0.34213 0.0143]',[-0.34068 -0.34262 0.0143]',[0.34407 -0.34229 0.0143]',[0.33898 0.33769 0.0913]',[-0.33624 0.33736 0.0913]',[-0.33591 -0.33785 0.0913]',[0.3393 -0.33753 0.0913]'];
% positions = [[0.34374 0.34374 0.0143]',[-0.34374 0.34374 0.0143]',[-0.34374 -0.34374 0.0143]',[0.34374 -0.34374 0.0143]',[0.34374 0.34374 0.0913]',[-0.34374 0.34374 0.0913]',[-0.34374 -0.34374 0.0913]',[0.34374 -0.34374 0.0913]'];
multirotor.setRotorPosition(1:8,positions);
% Define rotor orientations
orientations = [[-0.061628417 -0.061628417 0.996194698]',[0.061628417 -0.061628417 0.996194698]',[0.061628417 0.061628417 0.996194698]',[-0.061628417 0.061628417 0.996194698]',[-0.061628417 -0.061628417 0.996194698]',[0.061628417 -0.061628417 0.996194698]',[0.061628417 0.061628417 0.996194698]',[-0.061628417 0.061628417 0.996194698]'];
% orientations = [[0 0 1]',[0 0 1]',[0 0 1]',[0 0 1]',[0 0 1]',[0 0 1]',[0 0 1]',[0 0 1]'];
multirotor.setRotorOrientation(1:8,orientations);
% Define aircraft's inertia
multirotor.setMass(6.015);
mass = 6;
payloadRadius = 0.3*mean(sqrt(sum(positions.^2)));
multirotor.setPayload([0, 0, -payloadRadius],mass,eye(3)*2*mass*payloadRadius*payloadRadius/5);
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
        200
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
liftCoeff = 0.8*[0.00004
            0.00007
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
 dragCoeff = [0.0000005
            0.00000075
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
% multirotor.setRotorLiftCoeff(1:8,ones(1,8)*6.97e-5);
% multirotor.setRotorDragCoeff(1:8,ones(1,8)*1.033e-6);
% Define rotor inertia
multirotor.setRotorInertia(1:8,0.00047935*ones(1,8));
% Sets rotors rotation direction for control allocation
rotationDirection = [1 -1 1 -1 -1 1 -1 1]';
% rotationDirection = [1 1 1 1 -1 -1 -1 -1]';
multirotor.setRotorDirection(1:8,rotationDirection);
multirotor.setRotorMaxSpeed(1:8,729*ones(1,8));
multirotor.setRotorMinSpeed(1:8,0*ones(1,8));
multirotor.setInitialRotorSpeeds(328*rotationDirection);
multirotor.setInitialInput(9.47*rotationDirection);
multirotor.setInitialVelocity([0;0;0]);
multirotor.setInitialPosition([0;0;0]);
multirotor.setInitialAngularVelocity([0;0;0]);
multirotor.setRotorRm(1:8,0.0975*ones(1,8));
multirotor.setRotorKt(1:8,0.02498*ones(1,8));
multirotor.setRotorKv(1:8,340*ones(1,8));
multirotor.setRotorMaxVoltage(1:8,22*ones(1,8));
multirotor.setRotorOperatingPoint(1:8,340*[1 1 1 1 1 1 1 1]);

%% Previous case
% % Creates simulation class
% multirotor = multicontrol(8);
% multirotor.supressVerbose()
% % Define rotor positions (from rotor 1 to 8 in this order. using default rotor orientations)
% positions = [[0.25864 0 0.10]',[0 -0.25864 0.10]',[-0.25864 0 0.10]',[0 0.25864 0.10]',[0.25864 0 -0.10]',[0 -0.25864 -0.10]',[-0.25864 0 -0.10]',[0 0.25864 -0.10]'];
% multirotor.setRotorPosition([1 2 3 4 5 6 7 8],positions);
% % Define aircraft's inertia
% inertia = 1.5*[0.031671826 0 0;0 0.061646669 0;0 0 0.032310702];
% multirotor.setMass(1.85);
% multirotor.setInertia(inertia);
% % Define lift coefficients (using default drag coefficients)
% multirotor.setRotorLiftCoeff([1 2 3 4 5 6 7 8],0.00000289*[1 1 1 1 1 1 1 1]);
% % Sets rotors rotation direction for control allocation
% rotationDirection = [1 -1 1 -1 -1 1 -1 1]';
% multirotor.setRotorDirection([1 2 3 4 5 6 7 8],rotationDirection);
% multirotor.setRotorMaxSpeed(1:8,1200*[1 1 1 1 1 1 1 1])
% multirotor.setRotorOperatingPoint(1:8,340*[1 1 1 1 1 1 1 1]);

%% Controller configuration
% Trajectory controller
% For RLQ-R Passive, Active and Modified, SOSMC Passive and Active:
% kp = 400*[1 1 1];ki = 40*[1 1 1];kd = 100*[1 1 1];kdd = 1*[1 1 1];
% For SOSMC Direct:
% kp = 30*[1.8 1.8 0.9];ki = 10*[1 1 1];kd = 15*[0.5 0.5 0.5];kdd = 1*[1 1 1];
% For Adaptive Direct:
% kp = 15*[1.8 1.8 0.9];ki = 5*[1 1 1];kd = 20*[0.5 0.5 0.5];kdd = 1*[1 1 1];
% For SOSMC with PIDD:
% kp = 5*[1.8 1.8 1.8];ki = 0*[1 1 1];kd = 20*[0.5 0.5 0.5];kdd = 1*[1 1 1];
% For Adaptive with PIDD:

% For Adaptive CA:
% kp = 100*[1 1 1];ki = 20*[1 1 1];kd = 60*[1 1 1];kdd = 1*[1 1 1];
% For attitude PID:
%kp = [90 90 90];ki = [10 10 30];kd = [40 40 40];kdd = [2 2 2];
%kp = [0 0 0];ki = [0 0 0];kd = [0 0 0];kdd = [0 0 0];
kp = [40 40 100];ki = [2 2 40];kd = [5 5 70];kdd = [0 0 2];
multirotor.configController('Position PIDD',kp,ki,kd,kdd);

% PID attitude controller
kp = [130 130 50];
ki = [200 200 200];
kd = [14 14 2];
% PD for Adaptive CA
% kp = 2900*[1 1 1];
% ki = 200*[1 1 1];
% kd = 140*[1 1 1];
multirotor.configController('PID',kp,ki,kd);

% R-LQR attitude controller
%P = 10000*diag([2,2,4,.1,.1,.1]); % Variance of the state used in the robust control
% Q = 2*diag([1,0.01,0.5,1,6,2]);
% P = Q;
% R = 0.5*diag([1,.05,1]);
Q = diag([1e4, 1e4, 1e3,1e-1,1e-1,1e-1]);
P = Q;
R = 50*diag([1,1,1]);
Ef = 0.02*[1 1 1 0 0 0];
Eg = 0.02*[1 1 1];
H = [1 1 1 1 1 1]';
mu = 1e30;
alpha = 1.5;
multirotor.configController('RLQ-R Passive',P,Q,R,Ef,Eg,H,mu,alpha);
multirotor.configController('RLQ-R Active',P,Q,R,Ef,Eg,H,mu,alpha);
Q = 50000*blkdiag(1e1, 1e1, 1e1,1e1,1e1,1e1);
R = 0.000001*eye(8);
Ef = 0.2*[1 1 1 0 0 0];
Eg = 0.2*[1 1 1 1 1 1 1 1];
mu = 1e20;
alpha = 1.5;
multirotor.configController('RLQ-R Passive Modified',P,Q,R,Ef,Eg,H,mu,alpha);
multirotor.configController('RLQ-R Active Modified',P,Q,R,Ef,Eg,H,mu,alpha);
P = eye(9);
Q = 50000000*blkdiag(1e1, 1e1, 1e1, 1e1, 1e1, 1e1,1e1,1e1,1e1);
R = 0.000001*eye(8);
H = [1 1 1 1 1 1 1 1 1]';
Ef = 0.2*[1 1 1 1 1 1 0 0 0];
Eg = 0.2*[1 1 1 1 1 1 1 1];
mu = 1e20;
alpha = 1.5;
multirotor.configController('RLQ-R Passive Modified with PIDD',P,Q,R,Ef,Eg,H,mu,alpha);
multirotor.configController('RLQ-R Active Modified with PIDD',P,Q,R,Ef,Eg,H,mu,alpha);

% SOSMC controller
c = 0*diag([1,1,0.001]);
alpha =  0*diag([1,1,0.001]);
lambda = 0*diag([0.1,0.1,0.001]);
multirotor.configController('SOSMC Passive',c,lambda,alpha);
multirotor.configController('SOSMC Active',c,lambda,alpha);
c = 30*diag([1,1,0.001,2,2,2]);
alpha = .5*diag([1,1,0.001,2,2,2]);
lambda = 1*diag([0.1,0.1,0.001,2,2,2]);
multirotor.configController('SOSMC Passive with PIDD',c,lambda,alpha);
multirotor.configController('SOSMC Active with PIDD',c,lambda,alpha);
c =25*diag([3,3,3,1,1,1]);
alpha = 15000*...
           [ 1 -1  0.1 -0.001 -0.01 1
             1  1 -0.1  0.001 -0.01 1
            -1  1  0.1  0.001  0.01 1
            -1 -1 -0.1 -0.001  0.01 1
             1 -1 -0.1 -0.001 -0.01 1
             1  1  0.1  0.001 -0.01 1
            -1  1 -0.1  0.001  0.01 1
            -1 -1  0.1 -0.001  0.01 1];
lambda = 5000*...
            [1 -1  0.1 -0.001 -0.001 1
             1  1 -0.1  0.001 -0.001 1
            -1  1  0.1  0.001  0.001 1
            -1 -1 -0.1 -0.001  0.001 1
             1 -1 -0.1 -0.001 -0.001 1
             1  1  0.1  0.001 -0.001 1
            -1  1 -0.1  0.001  0.001 1
            -1 -1  0.1 -0.001  0.001 1];
multirotor.configController('SOSMC Passive Direct',c,lambda,alpha,1,0);
multirotor.configController('SOSMC Active Direct',c,lambda,alpha,1,0);

% Adaptive controller
Am = -10*diag([1,1,1]);
Q = 1e5*eye(3);
gamma1 = eye(3)*0.000005;
gamma2 = eye(3)*0.000005;
gamma3 = eye(3)*0.000005;
gamma4 = eye(3)*0.000005;
multirotor.configController('Adaptive',Am,Q,gamma1,gamma2,gamma3,gamma4);
Am = -10*diag([1,1,1,10,10,10]);
Q = 10e5*eye(6);
gamma1 = eye(6)*0.000005;
gamma2 = eye(6)*0.000005;
gamma3 = eye(6)*0.000005;
gamma4 = eye(6)*0.000005;
multirotor.configController('Adaptive with PIDD',Am,Q,gamma1,gamma2,gamma3,gamma4);
Am = -15*diag([.1,.1,.1,1,1,5]);
Q = 5e3*diag([1,1,1,40,40,40]);
gamma1 = eye(8)*100;
gamma2 = eye(8)*1;
gamma3 = eye(8)*10;
gamma4 = eye(8)*0.05;
B0 = 3e3*[ -0.00001 -0.00001 4  1.5 -1.5  -1 
             0.00001 -0.00001 4  1.5  1.5 1  
             0.00001  0.00001 4 -1.5  1.5  -1  
            -0.00001  0.00001 4 -1.5 -1.5 1 
            -0.00001 -0.00001 4  1.5 -1.5 1 
             0.00001 -0.00001 4  1.5  1.5  -1  
             0.00001  0.00001 4 -1.5  1.5 1  
            -0.00001  0.00001 4 -1.5 -1.5  -1];
multirotor.configController('Adaptive Direct',Am,Q,gamma1,gamma2,gamma3,gamma4,B0);

% Adaptive control allocation
multirotor.configControlAllocator('Adaptive',-1e14*eye(6),1,0);
multirotor.configControlAllocator('Passive NMAC',1,0);
multirotor.configControlAllocator('Active NMAC',1,0);

% Configure simulator
% multirotor.setRotorStatus(1,'stuck',0.5)
multirotor.setTimeStep(0.005);
multirotor.setControlTimeStep(0.05);
multirotor.setController('RLQ-R Passive');
multirotor.setControlAllocator('Passive NMAC');
multirotor.setAttitudeReferenceCA('Passive NMAC');
multirotor.configFDD(1,0.1)

% multirotor.setTrajectory('waypoints',[[1 1 1 0 0.4 0.4 0]',[1 2 3 0 0 0 0]',[1 2 3 0 0 0 pi/2]'],[5 10 15]);
multirotor.setTrajectory('waypoints',[30 30 30 pi/2]',10);
% xpos = linspace(0,1,1000);
% ypos = linspace(0,1,1000);
% zpos = linspace(0,1,1000);
% yawpos = linspace(0,pi/2,1000);
% time = linspace(0.1,10,1000);% 
% xvel = diff(xpos)./diff(time);
% xvel(end+1) = 0;
% yvel = diff(ypos)./diff(time);
% yvel(end+1) = 0;
% zvel = diff(zpos)./diff(time);
% zvel(end+1) = 0;
% yawvel = diff(yawpos)./diff(time);
% yawvel(end+1) = 0;
% multirotor.setTrajectory('waypoints',[xpos; ypos; zpos; xvel; yvel; zvel; yawpos; yawvel],time);
% multirotor.setTrajectory('gerono',7,4,4,30,'fixed',0);

% endTime = 15;
% % [waypoints, time] = geronoToWaypoints(7, 4, 4, endTime, endTime/8, 'sinusoidal',0,pi/2,endTime);
% [waypoints, time] = geronoToWaypoints(7, 4, 4, endTime, endTime/8, '360');
% multirotor.setTrajectory('waypoints',waypoints,time);

% multirotor.addCommand({'setRotorStatus(1,''stuck'',0.05)'},7)
% multirotor.addCommand({'setRotorStatus(1,''motor loss'',0.001)'},endTime/2)     
% multirotor.addCommand({'setRotorStatus(2,''motor loss'',0.001)'},endTime/2)
% multirotor.addCommand({'setRotorStatus(3,''motor loss'',0.001)'},endTime/2)
% multirotor.addCommand({'setRotorStatus(4,''motor loss'',0.001)'},endTime/2)
% multirotor.addCommand({'setRotorStatus(5,''motor loss'',0.75)'},endTime/2)
% multirotor.addCommand({'setRotorStatus(6,''motor loss'',0.75)'},endTime/2)
% multirotor.addCommand({'setRotorStatus(7,''motor loss'',0.75)'},endTime/2)
multirotor.setSimEffects('motor dynamics on','solver euler')
% multirotor.setLinearDisturbance('@(t) [0;1;0]*10*exp(-(t-3.75)^2/(0.5))')
multirotor.setControlDelay(0.20);
%% Run simulator
multirotor.run('visualizeGraph',false,'visualizeProgress',true,'metricPrecision',0.15,'angularPrecision',5,'endError',inf);
multirotor.plotSim();
% multirotor.plotAxes('rotorspeed',figure())
