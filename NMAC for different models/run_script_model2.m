% OK
% SKYF simulation
% refer to https://www.youtube.com/watch?v=CZ8ZFFjOzII

addpath('../multiControl/')
addpath('../multiControl/utils')
warning('off','all')
clear all
clc
%% Configuration for +8 coaxial octorotor
% Creates simulation class
multirotor = multicontrol(6);
% Define rotor positions
positions = [2.41, -2.41, -2.41,  2.41, 1.55, -1.55
             2.41,  2.41, -2.41, -2.41,    0,     0
              0.6,   0.6,   0.6,   0.6,  0.6,   0.6];
multirotor.setRotorPosition(1:6,positions);
% Define rotor orientations
orientations = [[0 0 1]',[0 0 1]',[0 0 1]',[0 0 1]',[0 0 1]',[0 0 1]'];
multirotor.setRotorOrientation(1:6,orientations);
% Define aircraft's inertia
multirotor.setMass(350);
inertia =   diag([157,61,146]);
multirotor.setInertia(inertia);
% Define aircraft's drag coeff
friction = [0.25	0	0
            0	0.25	0
            0	0	0.25];
multirotor.setFriction(friction);
multirotor.setAngularFilterGain([0,0,0.5]);
% Define lift and drag coefficients
% multirotor.setRotorLiftCoeff(1:6,[ones(1,4)*0.0017899323,ones(1,2)*0.0129371703]);
multirotor.setRotorLiftCoeff(1:6,[ones(1,4)*5.9664e-04,ones(1,2)*0.0229371703]);
% multirotor.setRotorDragCoeff(1:6,[ones(1,4)*7.5304514656581E-05,ones(1,2)*0.0006726965]);
multirotor.setRotorDragCoeff(1:6,[ones(1,4)*2.5102e-05,ones(1,2)*0.0006726965]);
% Define rotor inertia
multirotor.setRotorInertia(1:6,[ones(1,4)*0.023412776,ones(1,2)*7.8625708128]);
% Sets rotors rotation direction for control allocation
rotationDirection = [1 -1 1 -1 1 -1]';
multirotor.setRotorDirection(1:6,rotationDirection);
multirotor.setRotorMaxSpeed(1:6,[ones(1,4)*600,ones(1,2)*300]);
multirotor.setRotorMinSpeed(1:6,0*ones(1,6));
multirotor.setInitialRotorSpeeds(200*rotationDirection);
multirotor.setRotorOperatingPoint(1:6,[ones(1,4)*270,ones(1,2)*300]);

%% Controller configuration
% Trajectory controller
%PID:
kp = [300 700 100];
ki = [0 0 0];
kd = [2000 3000 2000];
kdd = [100 100 0];
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

endTime = 30;
[waypoints, time] = geronoToWaypoints(7, 4, 4, endTime, endTime/8, 'goto', 2*pi);
multirotor.setTrajectory('waypoints',waypoints,time);

multirotor.setSimEffects('motor dynamics off','solver euler')
multirotor.setControlDelay(0.20);
%% Run simulator
multirotor.run('visualizeGraph',false,'visualizeProgress',true,'metricPrecision',0.15,'angularPrecision',5,'endError',5);
multirotor.plotSim();
% multirotor.save('graphs');

numberOfRotors = 6;
figure
speed1 = [];
time1 = [];
names1 = [];
speed2 = [];
time2 = [];
names2 = [];
log = multirotor.log;
for it=1:numberOfRotors
    if mean(log.rotor(it).speed)>0
        names1 = [names1; ['Rotor ',num2str(it)]];
        time1 = [time1; log.time];
        speed1 = [speed1; log.rotor(it).speed];
    else
        names2 = [names2; ['Rotor ',num2str(it)]];
        time2 = [time2; log.time];
        speed2 = [speed2; log.rotor(it).speed];        
    end
end
subplot(2,1,1)
plot(time1',speed1')
ylabel('Rotor speeds (rad/s)')
legend(names1)
subplot(2,1,2)
plot(time2',speed2')
ylabel('Rotor speeds (rad/s)')
legend(names2)
xlabel('Time (s)')
