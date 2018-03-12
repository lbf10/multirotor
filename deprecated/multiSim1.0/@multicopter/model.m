function dydt = model(obj,t,y,simTime,simInput)
%MODEL calculates state-space equation for the multicopter model
%
%   Inputs are translated to rotor setpoints according to code below. Set
%   points are then translated to rotor speeds according to defined rotor
%   dynamics. In this version, mapping is direct and no rotor dynamics are
%   considered.
%
%   dydt = model(t,y,simTime,simInput) Returns a vector of derivatives of
%   the state vector y, at time t. simTime and simInput are auxiliary
%   variables to calculate input value at time t. simTime is the vector of
%   times for simulation and simInput their respective inputs at each time.
%   y must be a numeric column vector, t must be scalar, simTime must be a
%   numeric vector and simInput a numeric array.

    %%% Limits rotor speeds to maximum allowed speeds
    for i=1:obj.numberOfRotors_
        index = abs(simInput)>=obj.rotor_(i).maxSpeed;
        simInput(i,index(i,:)) = obj.rotor_(i).maxSpeed*(simInput(i,index(i,:))./abs(simInput(i,index(i,:))));
    end

    %%% Interpolates inputs because solver cannot deal with
    % discontinuities
    input = interp1(simTime, simInput',t);
    dSimInput = (1/obj.timeStep_)*diff([simInput simInput(:,end)]')';
    dinput = interp1(simTime, dSimInput',t);

    %%% Inputs to rotor set points
    % Used in case inputs are not necessarily rotor speeds
    for i=1:obj.numberOfRotors_
        obj.rotor_(i).speedSetPoint = input(i);
    end

    %%% Rotor dynamics
    % Relates rotor speeds to rotor set points
    % In this case, speeds are the same as set points
    w = zeros(obj.numberOfRotors_,1);
    dw = w;
    for i=1:obj.numberOfRotors_
        obj.previousState_.rotor(i).speed = obj.rotor_(i).speedSetPoint;
        w(i) = obj.previousState_.rotor(i).speed;
        dw(i) = dinput(i);
    end

    %%% Aircraft dynamics
    sum1 = zeros(3,1);
    sum2 = zeros(3,1);
    sum3 = zeros(3,1);
    for i=1:obj.numberOfRotors_
        sum1 = sum1 + obj.rotor_(i).inertia*w(i)*obj.rotor_(i).orientation;
        sum2 = sum2 + obj.rotor_(i).liftCoeff*cross(obj.rotor_(i).position,obj.rotor_(i).orientation)*w(i)^2;
        sum2 = sum2 - (obj.rotor_(i).dragCoeff*w(i)*abs(w(i))+obj.rotor_(i).inertia*dw(i))*obj.rotor_(i).orientation;
        sum3 = sum3 + obj.rotor_(i).liftCoeff*w(i)^2*obj.rotor_(i).orientation;
    end           

    Pa = y(1:3); % Position in relation to absolute reference frame
    q = y(4:7); % Attitude in quaternion
    Va = y(8:10); % Velocity in relation to absolute reference frame
    Wb = y(11:13); % Angular velocity in relation to body reference frame
    Sq = [-q(2),-q(3),-q(4);
          q(1),-q(4),q(3);
          q(4),q(1),-q(2);
          -q(3),q(2),q(1)];
    Q = obj.matrixBtoA(q); % Transformation matrix from body to absolute reference frame
    invQ = Q';

    % State-space equation
    dWb = obj.inertiaTensor_\(cross((obj.inertiaTensor_*Wb-sum1),Wb)+sum2);
    dVa = [0;0;-9.81]+(1/obj.mass_)*Q*(sum3-obj.translationalFriction_*invQ*Va);
    dq = 0.5*Sq*Wb;
    dPa = Va;

    dydt = [dPa; dq; dVa; dWb];            
end