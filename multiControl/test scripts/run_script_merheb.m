% Octorotor simulation
% refer to "doc multicontrol" or "doc multicopter" for more information

addpath('../../multiControl/')
addpath('../../multiControl/utils')
warning('off','all')
clear all
clc
%% Configuration for +8 coaxial octorotor
% Creates simulation class
multirotor = multicontrol(8);
% multirotor.supressVerbose()
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
mass = 0.4;
payloadRadius = 0.3*mean(sqrt(sum(positions.^2)));
multirotor.setPayload([0, 0, -payloadRadius],mass,eye(3)*2*mass*payloadRadius*payloadRadius/5);
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
% rotationDirection = [1 1 1 1 -1 -1 -1 -1]';
multirotor.setRotorDirection(1:8,rotationDirection);
multirotor.setRotorMaxSpeed(1:8,1500*ones(1,8));
multirotor.setRotorMinSpeed(1:8,0*ones(1,8));
multirotor.setInitialRotorSpeeds(328*rotationDirection);
multirotor.setInitialVelocity([0;0;0]);
multirotor.setInitialPosition([0;0;0]);
multirotor.setInitialAngularVelocity([0;0;0]);
multirotor.setRotorOperatingPoint(1:8,352*[1 1 1 1 1 1 1 1]);

%% Controller configuration
% Trajectory controller
% PID:
kp = [30 30 30];ki = [5 5 5];kd = [10 10 10];kdd = [0 0 0];
% RLQ-R Passive:
% kp = [40 40 40];ki = [10 10 10];kd = [20 20 20];kdd = [0 0 0];
% RLQ-R Passive Modified:
% kp = [40 40 40];ki = [10 10 10];kd = [20 20 20];kdd = [0 0 0];
% RLQ-R Passive Modified with PIDD:
% kp = [40 40 40];ki = [10 10 10];kd = [20 20 20];kdd = [0 0 0];
% SOSMC Passive and with PIDD:
% kp = [300 300 100];ki = [10 10 40];kd = [70 70 70];kdd = [35 35 2];
% SOSMC Passive and with PIDD:
%  kp = [30 30 30];ki = [5 5 5];kd = [10 10 10];kdd = [0 0 0];
% SOSMC Passive Direct:
% kp = [40 40 40];ki = [10 10 10];kd = [20 20 20];kdd = [0 0 0];
% SOSMC Active Direct:
% kp = [40 40 40];ki = [10 10 10];kd = [10 10 10];kdd = [0 0 0];
% % Adaptive:
% kp = [30 30 40];ki = [10 10 10];kd = [10 10 10];kdd = [0 0 0];
% Adaptive with PIDD:
% kp = [30 30 40];ki = [10 10 10];kd = [10 10 10];kdd = [0 0 0];
% Adaptive Direct:
% kp = [40 40 40];ki = [20 20 20];kd = [10 10 10];kdd = [0 0 0];
% Markovian passive:
% kp = [70 70 100];ki = [40 40 40];kd = [40 40 70];kdd = [15 15 2];
% kp = [0 0 0];ki = [0 0 0];kd = [0 0 0];kdd = [0 0 0];
% Markovian active:
% kp = [40 40 40];ki = [10 10 10];kd = [10 10 10];kdd = [0 0 0];
% kp = [0 0 0];ki = [0 0 0];kd = [0 0 0];kdd = [0 0 0];
multirotor.configController('Position PIDD',kp,ki,kd,kdd);

% PID attitude controller
kp = [400 400 400];
ki = [10 10 10];
kd = [50 50 50];
multirotor.configController('PID',kp,ki,kd);

% R-LQR attitude controller
Q = diag([1e4, 1e4, 1e4,1e-1,1e-1,1e-1]);
P = Q;
R = 70*diag([1,1,.1]);
Ef = 0.1*[2 2 1 0 0 0];
Eg = 0.02*[1 1 1];
H = [1 1 1 1 1 1]';
mu = 1e30;
alpha = 1.5;
multirotor.configController('RLQ-R Passive',P,Q,R,Ef,Eg,H,mu,alpha);
multirotor.configController('RLQ-R Active',P,Q,R,Ef,Eg,H,mu,alpha);
Q = 500000*blkdiag(1e1, 1e1, 1e1,1e1,1e1,1e1);
P = Q;
R = 0.00001*eye(8);
Ef = 10*[2 2 1 0 0 0];
Eg = 1000*[1 1 1 1 1 1 1 1];
H = [1 1 1 1 1 1]';
mu = 1e20;
alpha = 1.5;
multirotor.configController('RLQ-R Passive Modified',P,Q,R,Ef,Eg,H,mu,alpha);
multirotor.configController('RLQ-R Active Modified',P,Q,R,Ef,Eg,H,mu,alpha);
% P = eye(9);
Q = 500000*blkdiag(1e1, 1e1, 1e1, 1e1, 1e1, 1e1,1e1,1e1,1e1);
P = Q;
R = 0.00001*eye(8);
H = [1 1 1 1 1 1 1 1 1]';
Ef = 10*[1 1 1 1 1 1 0 0 0];
Eg = 40*[1 1 1 1 1 1 1 1];
mu = 1e22;
alpha = 1.5;
multirotor.configController('RLQ-R Passive Modified with PIDD',P,Q,R,Ef,Eg,H,mu,alpha);
multirotor.configController('RLQ-R Active Modified with PIDD',P,Q,R,Ef,Eg,H,mu,alpha);

% SOSMC controller
c = 3*diag([1,1,1]);
alpha =  2*diag([1,1,1]);
lambda = .1*diag([1,1,1]);
multirotor.configController('SOSMC Passive',c,lambda,alpha);
multirotor.configController('SOSMC Active',c,lambda,alpha);
c = 3*diag([1,1,1,2,2,2]);
alpha = 2*diag([1,1,1,2,2,2]);
lambda = .1*diag([1,1,1,2,2,2]);
multirotor.configController('SOSMC Passive with PIDD',c,lambda,alpha);
multirotor.configController('SOSMC Active with PIDD',c,lambda,alpha);
c =1.5*diag([1,1,1,2,2,2]);
alpha = 5500*...
           [ 1 -1  0.1 -0.001 -0.01 1
             1  1 -0.1  0.001 -0.01 1
            -1  1  0.1  0.001  0.01 1
            -1 -1 -0.1 -0.001  0.01 1
             1 -1 -0.1 -0.001 -0.01 1
             1  1  0.1  0.001 -0.01 1
            -1  1 -0.1  0.001  0.01 1
            -1 -1  0.1 -0.001  0.01 1];
lambda = 4000*...
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
Am = -2*diag([.1,.1,.1]);
Q = .01*diag([1,1,1]);
gamma1 = diag([1,1,1])*0.001;
gamma2 = diag([1,1,1])*0.001;
gamma3 = diag([1,1,1])*4;
gamma4 = diag([1,1,1])*0.0001;
multirotor.configController('Adaptive',Am,Q,gamma1,gamma2,gamma3,gamma4);
Am = -diag([.1,.1,2,2,2,0.01]);
Q = diag([1,1,1,.5,.5,.0005]);
gamma1 = diag([1,1,1,1,1,1])*.0001;
gamma2 = diag([1,1,1,1,1,1])*.0001;
gamma3 = diag([1,1,1,1,1,1])*4;
gamma4 = diag([1,1,1,1,1,1])*.00001;
multirotor.configController('Adaptive with PIDD',Am,Q,gamma1,gamma2,gamma3,gamma4);
Am = -1*diag([.1,.1,2,2,2,.01]);
Q = 1*diag([.1,.1,.1,.5,.5,.05]);
gamma1 = eye(8)*160;
gamma2 = eye(8)*160;
gamma3 = eye(8)*320;
gamma4 = eye(8)*25;
B0 = 5e3*[ -0.00001 -0.00001 4  1.5 -1.5  -6 
             0.00001 -0.00001 4  1.5  1.5 6  
             0.00001  0.00001 4 -1.5  1.5  -6  
            -0.00001  0.00001 4 -1.5 -1.5 6 
            -0.00001 -0.00001 4  1.5 -1.5 6 
             0.00001 -0.00001 4  1.5  1.5  -6  
             0.00001  0.00001 4 -1.5  1.5 6  
            -0.00001  0.00001 4 -1.5 -1.5  -6];
multirotor.configController('Adaptive Direct',Am,Q,gamma1,gamma2,gamma3,gamma4,B0);

% Markovian Passive Modified
P = eye(6);
modes = [1 1 1 1 1 1 1 1
         0 1 1 1 1 1 1 1
         0 0 1 1 1 1 1 1
         0 0 0 1 1 1 1 1
         0 0 0 0 1 1 1 1];
numberOfModes = size(modes,1);

Ef = [];
Ef(:,:,1) = 10*ones(1,6);
Ef(:,:,2) = 10*ones(1,6);
Ef(:,:,3) = 10*ones(1,6);
Ef(:,:,4) = 10*ones(1,6);
Ef(:,:,5) = 10*ones(1,6);

Eg = [];
Eg(:,:,1) = [1000 1000 1000 1000 1000 1000 1000 1000];
Eg(:,:,2) = [1000 1000 1000 1000 1000 1000 1000 1000];
Eg(:,:,3) = [1000 1000 1000 1000 1000 1000 1000 1000];
Eg(:,:,4) = [1000 1000 1000 1000 1000 1000 1000 1000];
Eg(:,:,5) = [1000 1000 1000 1000 1000 1000 1000 1000];

k = 1;

Eq = [];
Eq(:,:,1) = diag([1 1 1 1e-6 1e-6 1e-6]);
Eq(:,:,2) = diag([1 1 1 1e-6 1e-6 1e-6]);
Eq(:,:,3) = diag([1 1 1 1e-6 1e-6 1e-6]);
Eq(:,:,4) = diag([1 1 1 1e-6 1e-6 1e-6]);
Eq(:,:,5) = diag([1 1 1 1e-6 1e-6 1e-6]);

Er = [];
Er(:,:,1) = diag(0.000001*modes(1,:)+~modes(1,:));
Er(:,:,2) = diag(0.000001*modes(2,:)+~modes(2,:));
Er(:,:,3) = diag(0.000001*modes(3,:)+~modes(3,:));
Er(:,:,4) = diag(0.000001*modes(4,:)+~modes(4,:));
Er(:,:,5) = diag(0.000001*modes(5,:)+~modes(5,:));

lambda = 1;
pij = 0.5*eye(numberOfModes);
eij = 2*ones(numberOfModes, numberOfModes);
multirotor.configController('Markovian RLQ-R Passive Modified',modes,P,Ef,Eg,k,Er,Eq,lambda,pij,eij);

modes = [1 1 1 1 1 1 1 1
         0 1 1 1 1 1 1 1
         0 0 1 1 1 1 1 1
         0 0 0 1 1 1 1 1
         0 0 0 0 1 1 1 1];
numberOfModes = size(modes,1);
P = [];
for it = 1:numberOfModes
      P(:,:,it) = eye(6);  
end
Q = [];
Q(:,:,1) = 50000*blkdiag(1e10, 1e10, 1e1,1e-1,1e-1,1e-1);
Q(:,:,2) = 50000*blkdiag(1e10, 1e10, 1e1,1e-1,1e-1,1e-1);
Q(:,:,3) = 50000*blkdiag(1e10, 1e10, 1e1,1e-1,1e-1,1e-1);
Q(:,:,4) = 50000*blkdiag(1e10, 1e10, 1e1,1e-1,1e-1,1e-1);
Q(:,:,5) = 50000*blkdiag(1e10, 1e10, 1e1,1e-1,1e-1,1e-1);

R = [];
R(:,:,1) = diag(1*modes(1,:)+~modes(1,:));
R(:,:,2) = diag(1*modes(2,:)+~modes(2,:));
R(:,:,3) = diag(1*modes(3,:)+~modes(3,:));
R(:,:,4) = diag(1*modes(4,:)+~modes(4,:));
R(:,:,5) = diag(1*modes(5,:)+~modes(5,:));

Ef = [];
Ef(:,:,1) = 10000*ones(1,6);
Ef(:,:,2) = 10000*ones(1,6);
Ef(:,:,3) = 10000*ones(1,6);
Ef(:,:,4) = 10000*ones(1,6);
Ef(:,:,5) = 10000*ones(1,6);

Eg = [];
Eg(:,:,1) = [1000 1000 1000 1000 1000 1000 1000 1000];
Eg(:,:,2) = [1000 1000 1000 1000 1000 1000 1000 1000];
Eg(:,:,3) = [1000 1000 1000 1000 1000 1000 1000 1000];
Eg(:,:,4) = [1000 1000 1000 1000 1000 1000 1000 1000];
Eg(:,:,5) = [1000 1000 1000 1000 1000 1000 1000 1000];

H = [];
H(:,:,1) = [1 1 1 1 1 1]';
H(:,:,2) = [1 1 1 1 1 1]';
H(:,:,3) = [1 1 1 1 1 1]';
H(:,:,4) = [1 1 1 1 1 1]';
H(:,:,5) = [1 1 1 1 1 1]';

pij = 0.5*eye(numberOfModes);
ei = 2*ones(1,numberOfModes);
k = 1;
mu = 1e10;
alpha = 1.5;

multirotor.configController('Markovian RLQ-R Active Modified',modes,P,Q,R,Ef,Eg,H,pij,ei,k,mu,alpha);
multirotor.setAngularFilterGain([0 0 0.5]);

% Adaptive control allocation
multirotor.configControlAllocator('Adaptive',-1e14*eye(6),1,0);
multirotor.configControlAllocator('Passive NMAC',1,0);
multirotor.configControlAllocator('Active NMAC',1,0);

% Configure simulator
% multirotor.setRotorStatus(1,'stuck',0.5)
multirotor.setTimeStep(0.005);
multirotor.setControlTimeStep(0.05);
multirotor.setController('PID');
multirotor.setControlAllocator('Passive NMAC');
multirotor.setAttitudeReferenceCA('Passive NMAC');
multirotor.configFDD(.9,0.5)

% multirotor.setTrajectory('waypoints',[[1 1 1 0 0.4 0.4 0]',[1 2 3 0 0 0 0]',[1 2 3 0 0 0 pi/2]'],[5 10 15]);
% multirotor.setTrajectory('waypoints',[50 50 50 170*pi/180]',10);
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
% % 
endTime = 30;
% [waypoints, time] = geronoToWaypoints(7, 4, 4, endTime, endTime/8, 'goto',0);
[waypoints, time] = geronoToWaypoints(7, 4, 4, endTime, endTime/8, 'goto', 2*pi);
% [waypoints, time] = geronoToWaypoints(0, 0, 0, endTime, endTime/8, 'goto',0);
multirotor.setTrajectory('waypoints',waypoints,time);

% multirotor.addCommand({'setRotorStatus(1,''stuck'',0.05)'},7)
multirotor.addCommand({'setRotorStatus(1,''motor loss'',0.001)'},endTime/6)   
% multirotor.addCommand({'setRotorStatus(2,''prop loss'',0.001)'},endTime/2)    
% multirotor.addCommand({'setRotorStatus(3,''prop loss'',0.001)'},endTime/2)   
% multirotor.addCommand({'setRotorStatus(4,''prop loss'',0.001)'},endTime/2)  
% multirotor.addCommand({'setRotorStatus(2,''motor loss'',0.001)'},endTime/5)
% multirotor.addCommand({'setRotorStatus(3,''motor loss'',0.001)'},endTime/4)
% multirotor.addCommand({'setRotorStatus(4,''motor loss'',0.001)'},0)
% multirotor.addCommand({'setRotorStatus(5,''motor loss'',0.75)'},endTime/2)
% multirotor.addCommand({'setRotorStatus(6,''motor loss'',0.75)'},endTime/2)
% multirotor.addCommand({'setRotorStatus(7,''motor loss'',0.75)'},endTime/2)
multirotor.setSimEffects('motor dynamics off','solver euler')
multirotor.setLinearDisturbance('@(t) [0;1;0]*5*exp(-(t-3.75)^2/(0.5))')
multirotor.setControlDelay(0.20);
multirotor.setAngularFilterGain([0,0,0.5]);
%% Run simulator
multirotor.run('visualizeGraph',false,'visualizeProgress',true,'metricPrecision',0.15,'angularPrecision',5,'endError',5);
multirotor.plotSim();
% multirotor.plotAxes('rotorspeed',figure())
