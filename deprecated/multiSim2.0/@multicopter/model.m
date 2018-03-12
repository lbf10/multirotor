function dydt = model(obj,t,y,simTime,simInput)
%MODEL calculates state-space equation for the multicopter model
%
%   Inputs are translated to rotor setpoints according to code below. Set
%   points are then translated to rotor speeds according to defined rotor
%   transfer function.
%
%   dydt = model(t,y,simTime,simInput) Returns a vector of derivatives of
%   the state vector y, at time t. simTime and simInput are auxiliary
%   variables to calculate input value at time t. simTime is the vector of
%   times for simulation and simInput their respective inputs at each time.
%   y must be a numeric column vector, t must be scalar, simTime must be a
%   numeric vector and simInput a numeric array.

    % Utilizado na versao 1.0
    %%% Limits rotor speed inputs to maximum allowed speeds
    %     for i=1:obj.numberOfRotors_
    %         clear index;
    %         index = abs(simInput)>obj.rotor_(i).maxSpeed;
    %         simInput(i,index(i,:)) = obj.rotor_(i).maxSpeed;
    %         % Utilizado na versão 1.0
    %         %simInput(i,index(i,:)) = obj.rotor_(i).maxSpeed*(simInput(i,index(i,:))./abs(simInput(i,index(i,:))));
    %         clear index;
    %         index = abs(simInput)<obj.rotor_(i).minSpeed;
    %         simInput(i,index(i,:)) = obj.rotor_(i).minSpeed;
    %     end

    %%% Interpolates inputs because solver cannot deal with
    % discontinuities
    % Used in version 1.0
    % input = interp1(simTime, simInput',t);
    %%% Acts like a holder
    auxIndex = find(t<=simTime,1);
    input = simInput(:,auxIndex);
       
%     switch obj.simEffects_{1}
%         case 'motor dynamics tf on'
%             %does nothing
%         otherwise
%             dSimInput = (1/obj.timeStep_)*diff([simInput simInput(:,end)]')';
%             dinput = interp1(simTime, dSimInput',t);
%     end    
    %%% Inputs to rotor set points
    % Used in case inputs are not necessarily rotor speeds
    % or in case of faults
    for i=1:obj.numberOfRotors_
        switch obj.rotor_(i).status{2}
            case 'responding'
                obj.rotor_(i).speedSetPoint = input(i);
%                 switch obj.simEffects_{1}
%                     case 'motor dynamics tf on'
%                         % does nothing
%                     otherwise
%                         obj.rotor_(i).speedSetPoint = obj.rotor_(i).speedSetPoint*obj.rotor_(i).motorEfficiency;
%                 end
%                 if i>length(obj.log_.rotor)
%                     obj.log_.rotor(i).setPoint = zeros(1,length(obj.log_.time));
%                 end
%                 obj.log_.rotor(i).setPoint = [obj.log_.rotor(i).setPoint,obj.rotor_(i).speedSetPoint];
            otherwise
                % keep same set point as before
        end
    end    
%     for i=(obj.numberOfRotors_+1):length(obj.log_.rotor)
%         obj.log_.rotor(i).setPoint = [obj.log_.rotor(i).setPoint,0.0];
%     end

    %%% LOGs setPoints
    obj.spTimeAux_(end+1) = t;
    obj.setPointsAux_(:,end+1) = [obj.rotor_(1:obj.numberOfRotors_).speedSetPoint]';
    %%% Coefficients errors
    % Translates coefficients and their errors to auxiliary variables
    for i=1:obj.numberOfRotors_
        switch obj.simEffects_{1}
            case 'motor dynamics tf on'
                a(i) = (obj.rotor_(i).transferFunction(1)*(1+obj.rotor_(i).transferFunctionPError(1)));
                b(i) = (obj.rotor_(i).transferFunction(2)*(1+obj.rotor_(i).transferFunctionPError(2)))*(1/max(0.1,obj.rotor_(i).motorEfficiency));
                c(i) = obj.rotor_(i).transferFunction(3)*(1+obj.rotor_(i).transferFunctionPError(3));
            otherwise
                % does nothing
        end
        ri(i) = (obj.rotor_(i).inertia*(1+obj.rotor_(i).inertiaPError))*obj.rotor_(i).propEfficiency^2;
        lc(i) = (obj.rotorLiftCoeff(i)*(1+obj.rotor_(i).liftCoeffPError))*obj.rotor_(i).propEfficiency;
        dc(i) = (obj.rotorDragCoeff(i)*(1+obj.rotor_(i).dragCoeffPError))*obj.rotor_(i).propEfficiency;
        p(:,i) = obj.rotor_(i).position - obj.cgPositionError_;
    end
    m = obj.mass_*(1+obj.massPError_);
    I = obj.inertiaTensor_.*(eye(3)+obj.inertiaTensorPError_);
    A = obj.translationalFriction_.*(eye(3)+obj.translationalFrictionPError_);
    
    %%% Rotor dynamics
    % Relates rotor set points to rotor speeds
    % Limits rotor speed inputs to maximum allowed speeds
    for i=1:obj.numberOfRotors_
        localSetPoint(i) = obj.rotor_(i).speedSetPoint*obj.rotor_(i).motorEfficiency;
        if abs(localSetPoint(i))>obj.rotor_(i).maxSpeed;
            localSetPoint(i) = obj.rotor_(i).maxSpeed*sign(localSetPoint(i));
        end
        % Utilizado na versão 1.0
        %simInput(i,index(i,:)) = obj.rotor_(i).maxSpeed*(simInput(i,index(i,:))./abs(simInput(i,index(i,:))));
        if abs(localSetPoint(i))<obj.rotor_(i).minSpeed;
        	localSetPoint(i) = obj.rotor_(i).minSpeed*sign(localSetPoint(i));
        end
    end     
    switch obj.simEffects_{1}
        case 'motor dynamics tf on'
            w = y(14:(13+obj.numberOfRotors_));
            dw = y((14+obj.numberOfRotors_):(13+2*obj.numberOfRotors_));
            for i=1:obj.numberOfRotors_
                switch obj.rotor_(i).status{1}
                    case 'stuck'
                        dt = obj.rotor_(i).stuckTransitionPeriod;
                        e = 0.9;
                        nf = -log(0.02*sqrt(1-e^2))/(e*dt);
                        ddw(i,1) = -nf^2*w(i)-2*e*nf*dw(i);
                    otherwise
                        ddw(i,1) = -c(i)*w(i)-b(i)*dw(i)+a(i)*localSetPoint(i);
                end
            end
        otherwise            
            w = zeros(obj.numberOfRotors_,1);
            dw = w;
            for i=1:obj.numberOfRotors_
                switch obj.rotor_(i).status{1}
                    case 'stuck'
                        obj.previousState_.rotor(i).speed = 0;
                        w(i) = 0;
                    otherwise
                        obj.previousState_.rotor(i).speed = localSetPoint(i);
                        w(i) = obj.previousState_.rotor(i).speed;
                end  
            end
            obj.rotorSpeedsAux_(:,end+1) = w';
    end
    %%% Aircraft dynamics
    sum1 = zeros(3,1);
    sum2 = zeros(3,1);
    sum3 = zeros(3,1);
    for i=1:obj.numberOfRotors_
        sum1 = sum1 + ri(i)*w(i)*obj.rotor_(i).orientation;
        sum2 = sum2 + lc(i)*cross(p(:,i),obj.rotor_(i).orientation)*w(i)^2;
        sum2 = sum2 - (dc(i)*w(i)*abs(w(i))+ri(i)*dw(i))*obj.rotor_(i).orientation;
        sum3 = sum3 + lc(i)*w(i)^2*obj.rotor_(i).orientation;
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
    dWb = I\(cross((I*Wb-sum1),Wb)+sum2);
    if isempty(obj.linearDisturbance_)
        disturbance = [0;0;0];
    else
        fHandle = eval(obj.linearDisturbance_);
        disturbance = fHandle(t);
        disturbance = reshape(disturbance,[3,1]);
    end
    dVa = [0;0;-9.81]+(1/m)*Q*(sum3-A*invQ*Va)+disturbance/m;
    dq = 0.5*Sq*Wb;
    dPa = Va;

    dydt = [dPa; dq; dVa; dWb];  
    switch obj.simEffects_{1}
        case 'motor dynamics tf on'
            dydt = [dydt; dw; ddw]; 
        otherwise
            % does nothing
    end
end