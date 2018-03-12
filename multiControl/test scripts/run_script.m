% Octorotor simulation
% refer to "doc multicontrol" or "doc multicopter" for more information

addpath(genpath('../'))
warning('on','all')
 clear all
% close all
% clc
%% Configuration for +8 coaxial octorotor
% Creates simulation class
multirotor = multicontrol(8);
multirotor.supressVerbose()
% Define rotor positions (from rotor 1 to 8 in this order. using default rotor orientations)
positions = [[0.25864 0 0.10]',[0 -0.25864 0.10]',[-0.25864 0 0.10]',[0 0.25864 0.10]',[0.25864 0 -0.10]',[0 -0.25864 -0.10]',[-0.25864 0 -0.10]',[0 0.25864 -0.10]'];
multirotor.setRotorPosition([1 2 3 4 5 6 7 8],positions);
% Define aircraft's inertia
inertia = 1.5*[0.031671826 0 0;0 0.061646669 0;0 0 0.032310702];
multirotor.setMass(1.85);
multirotor.setInertia(inertia);
% Define lift coefficients (using default drag coefficients)
multirotor.setRotorLiftCoeff([1 2 3 4 5 6 7 8],0.00000289*[1 1 1 1 1 1 1 1]);
% Sets rotors rotation direction for control allocation
rotationDirection = [1 -1 1 -1 -1 1 -1 1]';
multirotor.setRotorDirection([1 2 3 4 5 6 7 8],rotationDirection);
multirotor.setRotorMaxSpeed(1:8,12000*[1 1 1 1 1 1 1 1])
% multirotor.setPayload([0;0;-0.1],1.85,zeros(3,3))

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
kp = 30*[1.8 1.8 1.8];ki = 3*[1 1 1];kd = 30*[0.5 0.5 0.5];kdd = 1*[1 1 1];
multirotor.configController('Position PIDD',kp,ki,kd,kdd);

% PD attitude controller
% kp = 300*[0.25 0.25 0.25];
% kd = 15*[0.25 0.25 0.25];
kp = 1000*[0.25 0.25 0.25];
ki = 100*[1 1 1];
kd = 90*[0.25 0.25 0.25];
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

% Configure simulator
% multirotor.setRotorStatus(1,'stuck',0.5)
multirotor.setControlTimeStep(0.1);
multirotor.setTimeStep(0.01);
multirotor.setController('Adaptive Direct');
multirotor.setControlAllocator('None');
multirotor.setAttitudeReferenceCA('PI Passive');
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
[waypoints, time] = geronoToWaypoints(7, 4, 4, endTime, endTime/8, 'sinusoidal',0,pi/2,endTime);
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
