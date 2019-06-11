% Octorotor simulation
% refer to "doc multicontrol" or "doc multicopter" for more information

addpath('../../multiControl/')
addpath('../../multiControl/utils')
warning('off','all')
clear all
clc
%% Configuration for Falcon 9 similar rocket
% Creates simulation class
multirotor = multicontrol(5);
% multirotor.supressVerbose()
% Define rotor positions
positions = [[1.85, 0, 23.33]',[0, 1.85, 23.33]',[-1.85, 0, 23.33]',[0, -1.85, 23.33]',[0, 0, -11.67]'];
multirotor.setRotorPosition(1:5,positions);
% Define rotor orientations
orientations = [[1 0 0]',[0 1 0]',[-1 0 0]',[0 -1 0]',[0 0 -1]'];
multirotor.setRotorOrientation(1:5,orientations);
% Define aircraft's inertia
multirotor.setMass(275000);
inertia = diag([43893421, 43893421, 470593]);
multirotor.setInertia(inertia);
% Define aircraft's drag coeff
friction = [0.25	0	0
            0	0.25	0
            0	0	0.25];
multirotor.setFriction(friction);
% Define lift and drag coefficients
multirotor.setRotorLiftCoeff(1:5,[20, 20, 20, 20, 275]);
multirotor.setRotorDragCoeff(1:5,zeros(1,5));
% Define rotor inertia
multirotor.setRotorInertia(1:5,zeros(1,5));
% Sets rotors rotation direction for control allocation
rotationDirection = [1 1 1 1 1]';
multirotor.setRotorDirection(1:5,rotationDirection);
multirotor.setRotorMaxSpeed(1:5,1000000000*ones(1,5));
multirotor.setRotorMinSpeed(1:5,0*ones(1,5));
multirotor.setInitialRotorSpeeds(328*rotationDirection);
multirotor.setInitialVelocity([0;0;0]);
multirotor.setInitialPosition([0;0;0]);
multirotor.setInitialAngularVelocity([0;0;0]);
multirotor.setRotorOperatingPoint(1:5,452*[1 1 1 1 1]);

%% Controller configuration
% Trajectory controller
% PID:
kp = [0 0 0];ki = [0 0 0];kd = [0 0 0];kdd = [0 0 0];
multirotor.configController('Position PIDD',kp,ki,kd,kdd);

% PID attitude controller
kp = [0 0 0];
ki = [0 0 0];
kd = [0 0 0];
multirotor.configController('PID',kp,ki,kd);

% Adaptive control allocation
% multirotor.configControlAllocator('Adaptive',-1e14*eye(6),1,0);
multirotor.configControlAllocator('Passive NMAC',1,0);
multirotor.configControlAllocator('Active NMAC',1,0);

% Configure simulator
% multirotor.setRotorStatus(1,'stuck',0.5)
multirotor.setTimeStep(0.005);
multirotor.setControlTimeStep(0.05);
multirotor.setController('PID');
multirotor.setControlAllocator('Passive NMAC');
multirotor.setAttitudeReferenceCA('Passive NMAC');

endTime = 60;
% [waypoints, time] = geronoToWaypoints(7, 4, 4, endTime, endTime/8, 'goto',0);
% [waypoints, time] = geronoToWaypoints(7, 4, 4, endTime, endTime/8, 'goto', 2*pi);
% [waypoints, time] = geronoToWaypoints(0, 0, 0, endTime, endTime/8, 'goto',0);
waypoints = [100,100,0,0]';
multirotor.setTrajectory('waypoints',waypoints,endTime);

multirotor.setSimEffects('motor dynamics off','solver ode45')
% multirotor.setLinearDisturbance(['@(t) [0;1;0]*5*(exp(-(t-',num2str(endTime/4),')^2/(0.5))+exp(-(t-',num2str(3*endTime/4),')^2/(0.5)))'])
multirotor.setControlDelay(0.20);
multirotor.setAngularFilterGain([0 0 0.9]);
%% Run simulator
multirotor.run('visualizeGraph',false,'visualizeProgress',true,'endError',100000000000);
multirotor.plotSim();
% multirotor.plotAxes('rotorspeed',figure())
