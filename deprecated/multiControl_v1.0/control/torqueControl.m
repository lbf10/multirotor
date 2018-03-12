function [tau, P, error] = torqueControl(T, qd,multirotor,controller)
%TORQUECONTROL Controller for attitude of aircraft
    %   tau = TORQUECONTROL(qd,multirotor,controller) calculates the necessary
    %   torque, in relation to body frame, for the aircraft to achieve the
    %   desired attitude qd. qd is a quaternion represented by a vector of
    %   length 4. multirotor is a multicopter object. controller is a struct 
    %   containing all the controller gains. tau is a vector of length 3
    %   indicating the torque in row, pitch and yaw axis, respectivelly.

    q = multirotor.previousAttitude(); %Current attitude
    % Quaternion error
    qe(1) = + qd(1)*q(1) + qd(2)*q(2) + qd(3)*q(3) + qd(4)*q(4);
    qe(2) = + qd(2)*q(1) - qd(1)*q(2) - qd(4)*q(3) + qd(3)*q(4);
    qe(3) = + qd(3)*q(1) + qd(4)*q(2) - qd(1)*q(3) - qd(2)*q(4);
    qe(4) = + qd(4)*q(1) - qd(3)*q(2) + qd(2)*q(3) - qd(1)*q(4);
    
    if strcmp(controller.type,'pd')
        errorVector = [qe(2); qe(3); qe(4)];
        tau = multirotor.inertia()*(-controller.PD.kd'.*multirotor.previousAngularVelocity()+controller.PD.kp'.*errorVector);
        P = [];
        error = [];
    elseif strcmp(controller.type,'rlqr')
        desiredAngularVelocity =(multirotor.toEuler(qd)-multirotor.toEuler(q))';
        desiredAngularVelocity(1) = normalize_angle(desiredAngularVelocity(1),-pi)/T;
        desiredAngularVelocity(2) = normalize_angle(desiredAngularVelocity(2),-pi)/T;
        desiredAngularVelocity(3) = normalize_angle(desiredAngularVelocity(3),-pi)/T;
        %desiredAngularVelocity = multirotor.matrixAbsoluteToBody()*desiredAngularVelocity;
        desiredAngularAcceleration = (desiredAngularVelocity - multirotor.previousAngularVelocity)/T;
        %desiredAngularVelocity = [0;0;0];
        desiredAngularAcceleration = [0;0;0];
        [tau, P, error] = robust_ddmr(T, qe, desiredAngularVelocity, desiredAngularAcceleration, multirotor, controller);
    else
        tau = [0;0;0];
    end
end

