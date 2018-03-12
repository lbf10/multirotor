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

    rotorIDs = 1:obj.numberOfRotors_;
    orientationMatrix = [obj.rotor_(rotorIDs).orientation];
    propEfficiencyMatrix = [obj.rotor_(rotorIDs).propEfficiency];
    % Utilizado na versao 1.0
    %%% Limits rotor speed inputs to maximum allowed speeds
    %     for i=rotorIDs
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
    for i=rotorIDs
        switch obj.rotor_(i).status{2}
            case 'responding'
                obj.rotor_(i).speedSetPoint = input(i);
            otherwise
                % keep same set point as before
        end
    end    

    %%% LOGs setPoints
    obj.spTimeAux_(end+1) = t;
    obj.setPointsAux_(:,end+1) = [obj.rotor_(rotorIDs).speedSetPoint]';
    %%% Coefficients errors
    % Translates coefficients and their errors to auxiliary variables
    switch obj.simEffects_{1}
            case 'motor dynamics tf on'  
                transferFunctions = [obj.rotor_(rotorIDs).transferFunction];
                tfErrors = [obj.rotor_(rotorIDs).transferFunctionPError];
                motorEfficiencies = [obj.rotor_(rotorIDs).motorEfficiency];
                a = (transferFunctions(1,:).*(1+tfErrors(1,:)));
                b = (transferFunctions(2,:).*(1+tfErrors(2,:))).*(1./max(0.1,motorEfficiencies));
                c = transferFunctions(3,:).*(1+tfErrors(3,:));
            otherwise
                % does nothing
    end
%     for i=rotorIDs
%         p(:,i) = obj.rotor_(i).position - obj.cgPositionError_;
%     end
%     p = bsxfun(@minus, [obj.rotor_(rotorIDs).position], obj.cgPositionError_);
    p = [obj.rotor_(rotorIDs).position] + repmat(obj.cgPositionError_,[1 obj.numberOfRotors_]);
    ri = ([obj.rotor_(rotorIDs).inertia].*(1+[obj.rotor_(rotorIDs).inertiaPError])).*(propEfficiencyMatrix.^2);
    lc = (obj.rotorLiftCoeff(rotorIDs).*(1+[obj.rotor_(rotorIDs).liftCoeffPError])).*propEfficiencyMatrix;
    dc = (obj.rotorDragCoeff(rotorIDs).*(1+[obj.rotor_(rotorIDs).dragCoeffPError])).*propEfficiencyMatrix;
    m = obj.mass_*(1+obj.massPError_);
    I = obj.inertiaTensor_.*(eye(3)+obj.inertiaTensorPError_);
    A = obj.translationalFriction_.*(eye(3)+obj.translationalFrictionPError_);
    
    %%% Rotor dynamics
    % Relates rotor set points to rotor speeds
    % Limits rotor speed inputs to maximum allowed speeds
    localSetPoint = [obj.rotor_(rotorIDs).speedSetPoint].*[obj.rotor_(rotorIDs).motorEfficiency];
    indexes = abs(localSetPoint(rotorIDs))>[obj.rotor_(rotorIDs).maxSpeed];
    if any(indexes)
        localSetPoint(indexes) = [obj.rotor_(indexes).maxSpeed].*sign(localSetPoint(indexes));
    end
    indexes = abs(localSetPoint(rotorIDs))<[obj.rotor_(rotorIDs).minSpeed];
    if any(indexes)
        localSetPoint(indexes) = [obj.rotor_(indexes).minSpeed].*sign(localSetPoint(indexes));
    end
%     for i=rotorIDs
%         localSetPoint(i) = obj.rotor_(i).speedSetPoint*obj.rotor_(i).motorEfficiency;
%         if abs(localSetPoint(i))>obj.rotor_(i).maxSpeed;
%             localSetPoint(i) = obj.rotor_(i).maxSpeed*sign(localSetPoint(i));
%         end
%         % Utilizado na versão 1.0
%         %simInput(i,index(i,:)) = obj.rotor_(i).maxSpeed*(simInput(i,index(i,:))./abs(simInput(i,index(i,:))));
%         if abs(localSetPoint(i))<obj.rotor_(i).minSpeed;
%         	localSetPoint(i) = obj.rotor_(i).minSpeed*sign(localSetPoint(i));
%         end
%     end     
    switch obj.simEffects_{1}
        case 'motor dynamics tf on'
            w = y(14:(13+obj.numberOfRotors_));
            dw = y((14+obj.numberOfRotors_):(13+2*obj.numberOfRotors_));
            ddw = dw;
            for i=rotorIDs
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
            for i=rotorIDs
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
    sum1 = orientationMatrix*(ri(rotorIDs)'.*w(rotorIDs));
    sum2 = (cross(p,orientationMatrix)*diag(lc)-orientationMatrix*diag(dc(rotorIDs)'.*sign(w)))*w.^2-orientationMatrix*(ri(rotorIDs)'.*dw(rotorIDs));
    sum3 = orientationMatrix*diag(lc(rotorIDs))*w.^2;

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