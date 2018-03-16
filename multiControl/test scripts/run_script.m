% Octorotor simulation
% refer to "doc multicontrol" or "doc multicopter" for more information

addpath(genpath('../'))
warning('on','all')
clear all
clc
%% Configuration for +8 coaxial octorotor
% Creates simulation class
multirotor = multicontrol(8);
multirotor.supressVerbose()
% Define rotor positions
% positions = [[0.34374 0.34245 0.0143]',[-0.341 0.34213 0.0143]',[-0.34068 -0.34262 0.0143]',[0.34407 -0.34229 0.0143]',[0.33898 0.33769 0.0913]',[-0.33624 0.33736 0.0913]',[-0.33591 -0.33785 0.0913]',[0.3393 -0.33753 0.0913]'];
positions = [[0.34374 0.34245 0.0143]',[-0.34374 0.34245 0.0143]',[-0.34374 -0.34245 0.0143]',[0.34374 -0.34245 0.0143]',[0.33898 0.33769 0.0913]',[-0.33898 0.33769 0.0913]',[-0.33898 -0.33769 0.0913]',[0.33898 -0.33769 0.0913]'];
multirotor.setRotorPosition(1:8,positions);
% Define rotor orientations
orientations = [[-0.061628417 -0.061628417 0.996194698]',[0.061628417 -0.061628417 0.996194698]',[0.061628417 0.061628417 0.996194698]',[-0.061628417 0.061628417 0.996194698]',[-0.061628417 -0.061628417 0.996194698]',[0.061628417 -0.061628417 0.996194698]',[0.061628417 0.061628417 0.996194698]',[-0.061628417 0.061628417 0.996194698]'];
%orientations = [[0 0 1]',[0 0 1]',[0 0 1]',[0 0 1]',[0 0 1]',[0 0 1]',[0 0 1]',[0 0 1]'];
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
%         multirotor.setRotorLiftCoeff(1:8,[speed liftCoeff],'poly2');
%         multirotor.setRotorDragCoeff(1:8,[speed dragCoeff],'poly2');
multirotor.setRotorLiftCoeff(1:8,mean(liftCoeff));
multirotor.setRotorDragCoeff(1:8,mean(dragCoeff));
% Define rotor inertia
multirotor.setRotorInertia(1:8,0.00047935*ones(1,8));
% Sets rotors rotation direction for control allocation
% rotationDirection = [1 -1 1 -1 -1 1 -1 1]';
rotationDirection = [1 1 1 1 -1 -1 -1 -1]';
multirotor.setRotorDirection(1:8,rotationDirection);
multirotor.setRotorMaxSpeed(1:8,750*ones(1,8));
multirotor.setRotorMinSpeed(1:8,0*ones(1,8));
% multirotor.setRotorMinSpeed(1:8,328*ones(1,8));

%% Controller configuration
% Trajectory controller
% For RLQ-R Passive, Active and Modified, SOSMC Passive and Active:
% kp = 15*[1.8 1.8 0.9];ki = 1*[1 1 1];kd = 15*[0.5 0.5 0.5];kdd = 1*[1 1 1];
% For SOSMC Direct:
% kp = 30*[1.8 1.8 0.9];ki = 10*[1 1 1];kd = 15*[0.5 0.5 0.5];kdd = 1*[1 1 1];
% For Adaptive Direct:
% kp = 15*[1.8 1.8 0.9];ki = 5*[1 1 1];kd = 20*[0.5 0.5 0.5];kdd = 1*[1 1 1];
% For SOSMC with PIDD:
% kp = 5*[1.8 1.8 1.8];ki = 0*[1 1 1];kd = 20*[0.5 0.5 0.5];kdd = 1*[1 1 1];
% For Adaptive with PIDD:
kp = 100*[1 1 1];ki = 20*[1 1 1];kd = 40*[1 1 1];kdd = 0*[1 1 1];
multirotor.configController('Position PIDD',kp,ki,kd,kdd);

% PD attitude controller
% kp = 300*[0.25 0.25 0.25];
% kd = 15*[0.25 0.25 0.25];
kp = 300*[1 1 1];
ki = 10*[1 1 1];
kd = 30*[1 1 1];
multirotor.configController('PID',kp,ki,kd);

% R-LQR attitude controller
P = 9e5*eye(6); % Variance of the state used in the robust control
Q = 40000*blkdiag(1e-1, 1e-1, 1e-1,1e1,1e1,1e1);
R = 10000*eye(3);
Ef = 0.2*[1 1 1 0 0 0];
Eg = 0.2*[1 1 1];
H = [1 1 1 1 1 1]';
mu = 1e30;
alpha = 1.5;
multirotor.configController('RLQ-R Passive',P,Q,R,Ef,Eg,H,mu,alpha);
multirotor.configController('RLQ-R Active',P,Q,R,Ef,Eg,H,mu,alpha);
Q = 50000*blkdiag(1e-1, 1e-1, 1e-1,1e1,1e1,1e1);
R = 0.000001*eye(8);
Ef = 0.2*[1 1 1 0 0 0];
Eg = 0.2*[1 1 1 1 1 1 1 1];
mu = 1e20;
alpha = 1.5;
multirotor.configController('RLQ-R Passive Modified',P,Q,R,Ef,Eg,H,mu,alpha);
multirotor.configController('RLQ-R Active Modified',P,Q,R,Ef,Eg,H,mu,alpha);
P = 9e5*eye(9);
Q = 50000*blkdiag(1e1, 1e1, 1e1, 1e-1, 1e-1, 1e-1,1e1,1e1,1e1);
R = 0.000001*eye(8);
H = [1 1 1 1 1 1 1 1 1]';
Ef = 0.2*[1 1 1 1 1 1 0 0 0];
Eg = 0.2*[1 1 1 1 1 1 1 1];
mu = 1e20;
alpha = 1.5;
multirotor.configController('RLQ-R Passive Modified with PIDD',P,Q,R,Ef,Eg,H,mu,alpha);
multirotor.configController('RLQ-R Active Modified with PIDD',P,Q,R,Ef,Eg,H,mu,alpha);

% SOSMC controller
c = 2*diag([1,1,0.001]);
alpha = 0.1*diag([1,1,0.001]);
lambda = 0.1*diag([0.1,0.1,0.001]);
multirotor.configController('SOSMC Passive',c,lambda,alpha);
multirotor.configController('SOSMC Active',c,lambda,alpha);
c = 0.5*diag([1,1,1,2,2,2]);
alpha = 0.2*diag([0.01,0.01,1,2,2,2]);
lambda = 0.01*diag([1,1,1,2,2,2]);
multirotor.configController('SOSMC Passive with PIDD',c,lambda,alpha);
multirotor.configController('SOSMC Active with PIDD',c,lambda,alpha);
c =0*diag([1,1,1,0,0,1]);
alpha = -180000*...
            [0  1  1 0 0 -1
             1  0 -1 0 0 -1
             0 -1  1 0 0 -1
            -1  0 -1 0 0 -1
             0  1 -1 0 0 -1
             1  0  1 0 0 -1
             0 -1 -1 0 0 -1
            -1  0  1 0 0 -1];
lambda = -60000*...
            [0  1  1 0 0 -1
             1  0 -1 0 0 -1
             0 -1  1 0 0 -1
            -1  0 -1 0 0 -1
             0  1 -1 0 0 -1
             1  0  1 0 0 -1
             0 -1 -1 0 0 -1
            -1  0  1 0 0 -1];
multirotor.configController('SOSMC Passive Direct',c,lambda,alpha);
multirotor.configController('SOSMC Active Direct',c,lambda,alpha);

% Adaptive controller
Am = -1*diag([1,1,1.5]);
Q = 150*eye(3);
gamma1 = eye(3)*0.0000005;
gamma2 = eye(3)*0.0000005;
gamma3 = eye(3)*0.0000005;
gamma4 = eye(3)*0.0000005;
multirotor.configController('Adaptive',Am,Q,gamma1,gamma2,gamma3,gamma4);
Am = -1*diag([1,1,1,1,1,1]);
Q = 10*eye(6);
gamma1 = eye(6)*0.0000005;
gamma2 = eye(6)*0.0000005;
gamma3 = eye(6)*0.0000005;
gamma4 = eye(6)*0.0000005;
multirotor.configController('Adaptive with PIDD',Am,Q,gamma1,gamma2,gamma3,gamma4);
Am = -eye(6);
Q = 5*eye(6);
gamma1 = eye(8)*10000000;
gamma2 = eye(8)*10000000;
gamma3 = eye(8)*10000000;
gamma4 = eye(8)*10000000;
multirotor.configController('Adaptive Direct',Am,Q,gamma1,gamma2,gamma3,gamma4);

% Adaptive control allocation
multirotor.configControlAllocator('Adaptive',-2e12*eye(6));
multirotor.configControlAllocator('Passive NMAC',1,0);

% Configure simulator
% multirotor.setRotorStatus(1,'stuck',0.5)
multirotor.setControlTimeStep(0.1);
multirotor.setTimeStep(0.001);
multirotor.setController('PID');
multirotor.setControlAllocator('Passive NMAC');
multirotor.setAttitudeReferenceCA('Passive NMAC');
multirotor.configFDD(1,0.1)
% multirotor.setTrajectory('waypoints',[[1 1 1 0 0.4 0.4 0]',[1 2 3 0 0 0 0]',[1 2 3 0 0 0 pi/2]'],[5 10 15]);
% multirotor.setTrajectory('waypoints',[0.2*[1 1 1 0]',0.4*[1 1 1 0]'],[15 30]);
% xpos = linspace(0,1,1000);
% ypos = linspace(0,1,1000);
% zpos = linspace(0,1,1000);
% yawpos = linspace(0,pi/2,1000);
% time = linspace(0.1,10,1000);
% 
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
endTime = 30;
% [waypoints, time] = geronoToWaypoints(7, 4, 4, endTime, endTime/8, 'sinusoidal',0,pi/2,endTime);
[waypoints, time] = geronoToWaypoints(7, 4, 4, endTime, endTime/8, '360');
multirotor.setTrajectory('waypoints',waypoints,time);
% multirotor.addCommand({'setRotorStatus(1,''motor loss'',0)'},5)
% multirotor.addCommand({'setRotorStatus(2,''motor loss'',0)'},5)
% multirotor.addCommand({'setRotorStatus(3,''motor loss'',0)'},5)
% multirotor.addCommand({'setRotorStatus(4,''motor loss'',0)'},5)
% multirotor.addCommand({'setRotorStatus(5,''motor loss'',0.75)'},5)
% multirotor.addCommand({'setRotorStatus(6,''motor loss'',0.75)'},5)
% multirotor.addCommand({'setRotorStatus(7,''motor loss'',0.75)'},5)
multirotor.setSimEffects('motor dynamics off')
multirotor.setSimEffects('solver euler')
% multirotor.setLinearDisturbance('@(t) [1;0;0]*3*exp(-(t-5)^2/(0.5))')

%% Run simulator
multirotor.run('visualizeGraph',false,'visualizeProgress',true,'metricPrecision',0.3,'angularPrecision',15);
multirotor.plotSim();
