
% Tilted octa simulation using parameters from hexa below
% refer to "Mathematical modeling and control of a hexacopter", Alaimo,
% A.,2013

addpath('../multiControl/')
addpath('../multiControl/utils')
warning('off','all')
clear all
clc
%% Configuration for +8 coaxial octorotor
% Creates simulation class
multirotor = multicontrol(8);
% Define rotor positions
angles = 2*pi*(0:45:359)/360;
positions = [0.2250    0.1591    0.0000   -0.1591   -0.2250   -0.1591   -0.0000    0.1591
             0         0.1591    0.2250    0.1591    0.0000   -0.1591   -0.2250   -0.1591
             0   0.1125    0   0.1125    0   0.1125    0   0.1125];
multirotor.setRotorPosition(1:8,positions);
% Define rotor orientations
orientations = [1         0         0.0000    0        -1         0         0         0     
                0         0         1         0         0.0000    0        -1         0     
                0         1         0         1         0         1         0         1];
multirotor.setRotorOrientation(1:8,orientations);
% Define aircraft's inertia
multirotor.setMass(0.468);
inertia =   diag([4.856e-3,4.856e-3,8.801e-3]);
multirotor.setInertia(inertia);
% Define aircraft's drag coeff
friction = [0.25	0	0
            0	0.25	0
            0	0	0.25];
multirotor.setFriction(friction);
multirotor.setAngularFilterGain([0,0,0.5]);
% Define lift and drag coefficients
multirotor.setRotorLiftCoeff(1:8,ones(1,8)*2.980e-6);
multirotor.setRotorDragCoeff(1:8,ones(1,8)*1.140e-7);
% Define rotor inertia
multirotor.setRotorInertia(1:8,3.357e-5*ones(1,8));
% Sets rotors rotation direction for control allocation
rotationDirection = [-1 -1 1 1 -1 -1 1 1]';
multirotor.setRotorDirection(1:8,rotationDirection);
multirotor.setRotorMaxSpeed(1:8,1805*ones(1,8));
multirotor.setRotorMinSpeed(1:8,0*ones(1,8));
multirotor.setInitialRotorSpeeds(92*rotationDirection);
multirotor.setRotorOperatingPoint(1:8,920*[1 1 1 1 1 1 1 1]);

%% Controller configuration
% Trajectory controller
%PID:
kp = [3 5 1];
ki = [0 0 0];
kd = [2 3 1];
kdd = [1 1 0];
multirotor.configController('Position PIDD',kp,ki,kd,kdd);

% PID attitude controller
kp = [30 30 15];
ki = [2 2 0];
kd = [5 5 1];
multirotor.configController('PID',kp,ki,kd);
multirotor.configControlAllocator('Passive NMAC',1,0);

% Configure simulator
multirotor.setTimeStep(0.005);
multirotor.setControlTimeStep(0.05);
multirotor.setController('PID');
multirotor.setControlAllocator('Passive NMAC');
multirotor.setAttitudeReferenceCA('Passive NMAC');

endTime = 20;
[waypoints, time] = geronoToWaypoints(7, 4, 4, endTime, endTime/8, 'goto', 0);
multirotor.setTrajectory('waypoints',waypoints,time);

multirotor.setSimEffects('motor dynamics off','solver euler')
multirotor.setControlDelay(0.20);
%% Run simulator
multirotor.run('visualizeGraph',false,'visualizeProgress',true,'metricPrecision',0.15,'angularPrecision',5,'endError',5);
multirotor.plotSim();
multirotor.save('graphs');

