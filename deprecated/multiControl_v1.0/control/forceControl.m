function [ desiredForce ] = forceControl(desiredPosition, position, controller)
    %FORCECONTROL Controler for translation of aircraft
    %   desiredForce = FORCECONTROL(desiredPosition, position, controller)
    %   calculates the translational force, in relation to the inertial frame,
    %   that the aircraft has to exerce in order to follow the desired path.
    %   desiredPosition and position are vectors of length 3, with the position
    %   where for the aircraft to achieve and the current aircraft position,
    %   respectively. controller is a struct containing all the controller
    %   gains. desiredForce is a vector of length 3 indicating the force in x,
    %   y and z inertial frame coordinates, respectively.

    forceError = desiredPosition-position;
    desiredForce = controller.force.kp'.*forceError+controller.force.kd'.*(forceError-previousForceError)/dt;
    previousForceError = forceError;
end

