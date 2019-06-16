% OK
% ennearotor simulation based on hexa
% refer to "Mathematical modeling and control of a hexacopter", Alaimo,
% A.,2013

addpath('../multiControl/')
addpath('../multiControl/utils')
warning('off','all')
clear all
clc
%% Configuration for +8 coaxial octorotor
% Creates simulation class
numberOfRotors = 9;
multirotor = multicontrol(numberOfRotors);
% Define rotor positions
insideAngles = 2*pi*(0:72:359)/360;
outsideAngles = [0, 90, 135, 180];
positions = [0.1125*cos(insideAngles) 0.3375*cos(outsideAngles)
             0.1125*sin(insideAngles) 0.3375*sin(outsideAngles)
             0*insideAngles 0*outsideAngles];
multirotor.setRotorPosition(1:numberOfRotors,positions);
% Define rotor orientations
orientations = [[0 0 1]',[0 0 1]',[0 0 1]',[0 0 1]',[0 0 1]',[0 0 1]',[0 0 1]',[0 0 1]',[0 0 1]'];
multirotor.setRotorOrientation(1:numberOfRotors,orientations);
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
multirotor.setRotorLiftCoeff(1:numberOfRotors,ones(1,numberOfRotors)*1.5e-6);
multirotor.setRotorDragCoeff(1:numberOfRotors,ones(1,numberOfRotors)*1e-7);
% Define rotor inertia
multirotor.setRotorInertia(1:numberOfRotors,6.357e-5*ones(1,numberOfRotors));
% Sets rotors rotation direction for control allocation
rotationDirection = [-1 1 -1 1 -1 1 -1 1 -1]';
multirotor.setRotorDirection(1:numberOfRotors,rotationDirection);
multirotor.setRotorMaxSpeed(1:numberOfRotors,2000*ones(1,numberOfRotors));
multirotor.setRotorMinSpeed(1:numberOfRotors,0*ones(1,numberOfRotors));
multirotor.setInitialRotorSpeeds(92*rotationDirection);
multirotor.setRotorOperatingPoint(1:numberOfRotors,920*[1 1 1 1 1 1 1 1 1]);

%% Controller configuration
% Trajectory controller
%PID:
kp = [4 6 2];
ki = [1 1 1];
kd = [3 4 2];
kdd = [1 1 0];
multirotor.configController('Position PIDD',kp,ki,kd,kdd);

% PID attitude controller
kp = [50 50 30];
ki = [10 30 10];
kd = [7 7 3];
multirotor.configController('PID',kp,ki,kd);
multirotor.configControlAllocator('Passive NMAC',1,0);

% Configure simulator
multirotor.setTimeStep(0.005);
multirotor.setControlTimeStep(0.05);
multirotor.setController('PID');
multirotor.setControlAllocator('Passive NMAC');
multirotor.setAttitudeReferenceCA('Passive NMAC');
multirotor.configFDD(1,0.25)

endTime = 15;
[waypoints, time] = geronoToWaypoints(7, 4, 4, endTime, endTime/8, 'goto', 2*pi);
multirotor.setTrajectory('waypoints',waypoints,time);

multirotor.addCommand({'setRotorStatus(1,''motor loss'',0.001)'},endTime/2)

multirotor.setSimEffects('motor dynamics off','solver euler')
multirotor.setControlDelay(0.20);
%% Run simulator
multirotor.run('visualizeGraph',false,'visualizeProgress',true,'metricPrecision',0.15,'angularPrecision',5,'endError',5);
multirotor.plotSim();
% multirotor.save('figures');
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

