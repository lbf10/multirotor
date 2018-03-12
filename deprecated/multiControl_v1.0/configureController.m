
previousVelocity = [0 0 0]';

%% Trajectory controller
controller.force.kp = 20*[1.8 1.8 0.9];
controller.force.kd = 10*[0.5 0.5 0.5];
controller.force.kdd = [1 1 1];

%% PD controller
controller.PD.kp = 300*[0.25 0.25 0.25];
controller.PD.kd = 15*[0.25 0.25 0.25];

%% R-LQR controller
controller.RLQR.P = 9e5*eye(6); % Variance of the state used in the robust control
controller.RLQR.Q = 10*blkdiag(1e-1, 1e-1, 1e-1,1e1,1e1,1e1);
controller.RLQR.R = 5000*eye(3);
controller.RLQR.Ef = 0.2*[1 1 1 0 0 0];
controller.RLQR.Eg = 0.2*[1 1 1];
controller.RLQR.numberOfIteractions = 150;