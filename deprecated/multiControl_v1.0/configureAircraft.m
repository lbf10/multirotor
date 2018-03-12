% Configure aircraft to simulate
% refer to "doc multicopter" for more information

%% Configuration for +8 coaxial octorotor
% Creates simulation class
multirotor = multicopter(8);
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
% Configure simulator
multirotor.setTimeStep(0.01);

% %% Configuration for +4 quadrotor
% % Creates simulation class
% multirotor = multicopter(4);
% % Define rotor positions (from rotor 1 to 8 in this order. using default rotor orientations)
% positions = [[0.25864 0 0]',[0 -0.25864 0]',[-0.25864 0 0]',[0 0.25864 0]'];
% multirotor.setRotorPosition([1 2 3 4],positions);
% % Define aircraft's inertia
% inertia = [0.031671826 0 0;0 0.061646669 0;0 0 0.032310702];
% multirotor.setMass(1.2211);
% multirotor.setInertia(inertia);
% % Define lift coefficients (using default drag coefficients)
% multirotor.setRotorLiftCoeff([1 2 3 4],0.00000289*[1 1 1 1]);
% % Sets rotors rotation direction for control allocation
% rotationDirection = [1 -1 1 -1]';
% % Configure simulator
% multirotor.setTimeStep(0.01);