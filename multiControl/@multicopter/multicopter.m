classdef multicopter < handle
    %MULTICOPTER Simulation class for a multicopter of arbitrary number of
    %rotors. Model version 2.0 
    %
    %   University of S�o Paulo - USP
    %   Author: Leonardo Borges Far�oni                       
    %   e-mail: leonardo.farconi@gmail.com                    
    %   Professor Advisor: Marco H. Terra and Roberto Inoue   
    %   E-mail: terra@sc.usp.br and rsinoue@ufscar.br         
    %   Date: October 9th, 2017                                 
    %   Revision 1:                                           
    %
    %   Model and class considerarions/hypothesis:
    %       - Body frame centered at CG/CM
    %       - Arbitrary number of rotors
    %       - Arbitrary rotor positions
    %       - Arbitrary rotor orientations
    %       - No blade-flapping
    %       - Quaternion modelling allowing any kind of rotations
    %       - Only translational aerodynamic drag
    %       - Lift and drag proportional to square of rotor speed 
    %       - Rotors variables can be changed during run time, but they are
    %       considered as constants and not dynamic variables in the model.
    %       - Euler angles are [row, pitch, yaw] always in this order,
    %       assuming a (1,2,3) rotation sequence, or in other words, the
    %       nautical and aeronautical convention. Reference: Representing 
    %       Attitude: Euler Angles, Unit Quaternions, and Rotation Vectors,
    %       J. Diebel, 2006.
    %       - Inputs are translated to rotor speeds based on a rotor
    %       transfer function from (Mohammadi 2013)
    %       - Rotors create lift only to the direction specified by the
    %       rotor orientation (rotors are not revertible)
    %
    %   This class simulates any kind of multicopter under
    %   the considerations above. Its intended use is to evaluate control
    %   and estimation algorithms. See example on run() function
    %   description.
    %
    %   To create a multicopter object, call the multicopter constructor,
    %   add the necessary number of rotors and set parameters using set
    %   functions. The aircraft dynamics and input maping are defined by 
    %   the private function model().
    %   
    %   Parameters can be set and get individually or in batches. Faults
    %   and uncertainties can be set using specific functions.
    %   Uncertainties are expressed by percentage errors in most cases.
    %   Use "doc multicopter" for methods listing and more information.
    %   
    %   See also SETMASS, SETINERTIA, SETFRICTION, ADDROTOR, SETTIMESTEP,
    %   SETINITIALSTATE, STARTLOGGING, RUN, RESET.
    
    properties (Access = protected)
        %%% Aircraft parameters
        mass_                   % Aircraft's total mass
        inertiaTensor_          % 3x3 matrix with inertia products in relation to body reference frame
        translationalFriction_  % 3x3 diagonal matrix with friction in each direction in relation to body reference frame
        numberOfRotors_         % scalar for number of either active or fail rotors
        rotor_                  % struct containing all rotor parameters and states - position and orientation in relation to body frame, inertia in relation to rotation axis, thrust coefficient, torque coefficient, status flag, efficiency, speed
        % Parameters' errors
        massPError_                     % Mass percentage error in relation to nominal mass
        translationalFrictionPError_    % Friction percentage error in relation to nominal friction
        inertiaTensorPError_            % Inertia percentage error in relation to nominal inertia tensor
        cgPositionError_                % Absolute position error for the CG in relation to nominal CG
        
        %%% Simulation parameters
        timeStep_           % scalar for simulation time step
        initialState_       % struct containing the initial state (state is comprised by the aircraft's states and rotor states in case )
        previousState_      % struct containing the previous state, previous input and previous time
        log_                % struct of arrays containing all state, input and time values from simulation start if the class is logging
        isLogging_          % flag that indicates whether simulation should be logged or not   
        isRunning_          % flag that indicates whether simulation is running or not
        opts_               % solver configuration
        solver_             % ODE solver
        simEffects_         % simulation effects selection
        linearDisturbance_  % string describing an anonymous function of time for a 3D force
        verbose_            % boolean allowing or not printing 
        
        % Auxiliary variables
        setPointsAux_
        rotorSpeedsAux_
        rotorAccAux_
        spTimeAux_
        dydtAux_
        % Rotor fail auxiliary variables
        % Rotor stuck function
%         Lu
%         Ld
%         C
%         D
    end
    
    methods        
        %%% Constructor
        function obj = multicopter(N,varargin)
        %MULTICOPTER Creates a multicopter object for simulation.
        %
        %   MULTICOPTER() Creates an empty multicopter object with initial
        %   position and velocities set to zero. Call set functions to 
        %   configure multicopter for simulation.
        %
        %   MULTICOPTER(N) Creates a default multicopter with N rotors
        %   equally distributed and ready for simulation, based on
        %   parameters from "Modelling and control of quadcopter",
        %   Luukonen, 2011. Distributes rotors equally over xy plane in X
        %   configuration. The first rotor is always the first to the
        %   CW direction after the x axis in the xy plane. All other
        %   rotors are added in CCW direction starting from rotor 1.
        %
        %   See also SETMASS, SETINERTIA, SETFRICTION, ADDROTOR, SETROTOR,
        %   SETTIMESTEP, SETINITIALSTATE, STARTLOGGING, RUN, RESET.
        
            obj.verbose_ = true;
            if nargin >= 1
                if ~isempty(varargin)
                    if strcmp(varargin{1},'-v')
                        obj.verbose_ = true;
                    elseif strcmp(varargin{1},'-nv')
                        obj.verbose_ = false;
                    end
                end
                if isnumeric(N) && N>0 && rem(N,1)==0
                    obj.numberOfRotors_ = N;
                    rotorMass = 0.468/4;
                    obj.mass_ = N*(rotorMass);    
                    obj.translationalFriction_ = eye(3)*0.25;
                    obj.inertiaTensor_ = [0,0,0;0,0,0;0,0,0];
                    angle = 2*pi/N;
                    angles = [-angle/2,-angle/2+angle*[1:N-1]];
                    l = 0.225;
                    for it=1:N
                        obj.rotor_(it).position = l*[cos(angles(it));sin(angles(it));0];
                        obj.inertiaTensor_ = obj.inertiaTensor_+rotorMass*[obj.rotor_(it).position(2)^2, obj.rotor_(it).position(1)*obj.rotor_(it).position(2), 0;
                                                                         obj.rotor_(it).position(2)*obj.rotor_(it).position(1), obj.rotor_(it).position(1)^2, 0;
                                                                         0, 0 ,l^2];
                        obj.rotor_(it).orientation = [0;0;1];
                        obj.rotor_(it).inertia = 3.357e-5;
                        obj.rotor_(it).liftCoeff.data = 2.98e-6;
                        obj.rotor_(it).dragCoeff.data = 1.14e-7;
                        obj.rotor_(it).liftCoeff.fit = [];
                        obj.rotor_(it).dragCoeff.fit = [];
                        obj.rotor_(it).status = {'free','responding','motor ok','prop ok'};
                        obj.rotor_(it).motorEfficiency = 1;
                        obj.rotor_(it).propEfficiency = 1;
                        obj.rotor_(it).inputSetPoint = 0;
                        obj.rotor_(it).maxSpeed = inf;
                        obj.rotor_(it).minSpeed = 0;
                        obj.rotor_(it).transferFunction = [250000;750;250000];
                        obj.rotor_(it).Rm = 0.0975;
                        obj.rotor_(it).Kt = 0.02498;
                        obj.rotor_(it).Kv = 340;
                        obj.rotor_(it).Io = 0.6/10;
                        obj.rotor_(it).maxVoltage = 22;
                        obj.initialState_.rotor(it).speed = 0;
                        obj.initialState_.rotor(it).acceleration = 0;
                        
                        obj.rotor_(it).inertiaPError = 0;
                        obj.rotor_(it).liftCoeffPError = 0;
                        obj.rotor_(it).dragCoeffPError = 0;
                        obj.rotor_(it).transferFunctionPError = [0;0;0];  
                        
                        obj.rotor_(it).stuckTransitionPeriod = []; 
                    end                    
                    obj.initialState_.input = zeros(obj.numberOfRotors_,1);
                else
                    warning('\n Number of rotors must be numeric, greater than  zero and integer! \n Configuring empty multirotor...')
                    obj.numberOfRotors_ = 0;
                    obj.mass_ = [];
                    obj.inertiaTensor_ = [];
                    obj.rotor_ = [];
                    obj.translationalFriction_ = [];

                    obj.initialState_.rotor = [];
                    obj.initialState_.input = [];
                end
            else
                obj.numberOfRotors_ = 0;
                obj.mass_ = [];
                obj.inertiaTensor_ = [];
                obj.rotor_ = [];
                obj.translationalFriction_ = [];
                
                obj.initialState_.rotor = [];
                obj.initialState_.input = [];
            end
            obj.timeStep_ = 0.05;
                
            obj.massPError_ = 0;
            obj.inertiaTensorPError_ = [0,0,0;0,0,0;0,0,0];  
            obj.translationalFrictionPError_ = zeros(3);              
            obj.cgPositionError_ = [0;0;0];

            obj.initialState_.position = [0;0;0];
            obj.initialState_.attitude = [1;0;0;0];
            obj.initialState_.velocity = [0;0;0];
            obj.initialState_.angularVelocity = [0;0;0];
            obj.initialState_.acceleration = [0;0;0];
            obj.initialState_.angularAcceleration = [0;0;0];
            
            obj.previousState_ = obj.initialState_;
            obj.previousState_.time = 0;
                    
            obj.isLogging_ = 1;
            obj.log_ = [];
            obj.log_.time = [];
            obj.log_.position = [];
            obj.log_.attitude = [];
            obj.log_.velocity = [];
            obj.log_.angularVelocity = [];
            obj.log_.input = [];
            for it=1:obj.numberOfRotors_
                obj.log_.rotor(it).setPoint = [];
                obj.log_.rotor(it).speed = [];
                obj.log_.rotor(it).acceleration = [];
                obj.log_.rotor(it).torque = [];
            end
            obj.log_.power = [];
            obj.isRunning_ = false;
            
            obj.simEffects_ = {'motor dynamics tf on','solver ode45'};
                
            obj.opts_ = odeset('RelTol',1e-2,'AbsTol',1e-4);
            
            obj.setPointsAux_ = [];
            obj.rotorSpeedsAux_ = [];
            obj.rotorAccAux_ = [];
            obj.spTimeAux_ = [];
            obj.linearDisturbance_ = [];
%             obj.Lu = 0.98;
%             obj.Ld = 0.02;
%             obj.C = atanh(1-2*obj.Lu)-atanh(1-2*obj.Ld);
%             obj.D = atanh(1-2*obj.Lu);
        end   
        
        %%% Set Functions
        % General set
        function setMass(obj, mass)
        %SETMASS  Sets mass of multicopter.
        %
        %   SETMASS(m) Sets the mass value for multicopter object.
        %   m must be scalar, numeric and greater than zero.
        %
        %   SETMASS can be used during simulation, but mass is considered a
        %   constant and not a dynamic variable for this model version. 
        %   Suggested use during simulation is for control robustness
        %   evaluation.
        %
        %   See also SETINERTIA, SETFRICTION, ADDROTOR.
        
            if ((isnumeric(mass)) & (mass>0) & (isscalar(mass)))
                obj.mass_ = mass;
                if obj.verbose_ == true
                    disp(['Mass set to: ',num2str(obj.mass_),' kg'])
                end
            else
                error('Mass must be numeric, greater than zero and scalar!')
            end
        end       
        function setInertia(obj, inertia)
        %SETINERTIA  Sets inertia tensor of multicopter.
        %
        %   SETINERTIA(I) Sets the inertia tensor for multicopter object.
        %   I must be numeric, related to the body reference frame and:
        %       - Column or row vector of length 3, with I(1), I(2) and
        %       I(3) equivalent to moments of inertia around x, y and z 
        %       body axis, respectively; or
        %       - Symmetric matrix of size 3x3, with I(1,1), I(2,2) and
        %       I(3,3) equivalent to moments of inertia around x, y and z
        %       body axis, respectively. I may not necessarily be diagonal.
        %
        %   SETINERTIA can be used during simulation, but inertia is 
        %   considered a constant and not a dynamic variable for this 
        %   model version. 
        %   Suggested use during simulation is for control robustness
        %   evaluation.
        %
        %   See also SETMASS, SETFRICTION, ADDROTOR.
        
            if isnumeric(inertia) && (isequal(size(inertia),[1 3]) || isequal(size(inertia),[3 1]) || (isequal(size(inertia),[3 3])))
                if ~isequal(size(inertia),[3 3])
                    obj.inertiaTensor_ = [inertia(1) 0 0; 0 inertia(2) 0; 0 0 inertia(3)];
                else
                    obj.inertiaTensor_ = inertia;
                end
                if obj.verbose_ == true
                    disp('Inertia tensor set to:')
                    disp(obj.inertiaTensor_)
                end
            else
                error('Inertia matrix must be numeric, have at least 3 values and be symmetric in case it is a tensor!')
            end
        end        
        function setFriction(obj, friction)    
        %SETFRICTION  Sets translational body drag coefficients.
        %
        %   SETFRICTION(A) Sets the translational body aerodynamic drag 
        %   coefficients for multicopter object.
        %   A must be numeric, related to the body reference frame and:
        %       - Column or row vector of length 3, with A(1), A(2) and
        %       A(3) equivalent to drag coefficients in x, y and z body 
        %       axis directions, respectively; or
        %       - Matrix of size 3x3, with A(1,1), A(2,2) and
        %       A(3,3) equivalent to drag coefficients in x, y and z body
        %       axis directions, respectively. Other elements of A may not
        %       necessarily be zero.
        %
        %   SETFRICTION can be used during simulation, but drag coefficients
        %   are considered constant and not dynamic variables for this 
        %   model version. 
        %   Suggested use during simulation is for control robustness
        %   evaluation.
        %
        %   See also SETMASS, SETINERTIA, ADDROTOR.
        
            if isnumeric(friction) && (isequal(size(friction),[1 3]) || isequal(size(friction),[3 1]) || isequal(size(friction),[3 3]))
                if ~isequal(size(friction),[3 3])
                    obj.translationalFriction_ = [friction(1) 0 0; 0 friction(2) 0; 0 0 friction(3)];
                else
                    obj.translationalFriction_ = friction;
                end
                if obj.verbose_ == true
                    disp('Translational body drag coefficients set to:')
                    disp(obj.translationalFriction_)
                end
            else
                error('Translation friction matrix must be numeric and have at least 3 values!');
            end
        end
        function setLinearDisturbance(obj, functionString)
        %SETLINEARDISTURBANCE Sets a function of time for a 3D force
        %disturbance
        %
        %   SETLINEARDISTURBANCE(functionString) Sets a function of time
        %   for a 3D force disturbance vector to be applied on the
        %   multirotor. The functionString must be a string defining an
        %   anonymous function with one input being the time and one 3D
        %   vector output being force.
        %   Example: functionString = '@(t) [1;0;0]*0.1*exp(-(t-5)^2/(4))'
        
            if ischar(functionString)
                obj.linearDisturbance_ = functionString;
                if obj.verbose_ == true
                    disp(['Linear disturbance function set to: ',obj.linearDisturbance_]);
                end
            else
                error('Linear disturbance must be a string defining an anonymous function.'); 
            end
        end
        
        function setMassError(obj, massError)
        %SETMASSERROR  Sets the percentage error in the multicopter`s mass
        %
        %   SETMASSERROR(m) Sets the mass percentage error for multicopter object.
        %   m must be scalar, numeric and can be either positive or negative.
        %   m = 0.1 means that the multicopter mass will be 10% greater
        %   than the value specified in setMass().
        %
        %   See also SETINERTIA, SETFRICTION, ADDROTOR.
        
            if isnumeric(massError) && isscalar(massError)
                obj.massPError_ = massError;
                if obj.verbose_ == true
                    disp(['Mass error set to: ',num2str(100*obj.massPError_),' %'])
                end
            else
                error('Mass error must be numeric and scalar!')
            end
        end   
        function setInertiaError(obj, inertiaError)
        %SETINERTIAERROR  Sets inertia percentage error tensor of multicopter.
        %
        %   SETINERTIAERROR(I) Sets the inertia percentage error tensor for multicopter object.
        %   I must be numeric, related to the body reference frame and:
        %       - Column or row vector of length 3, with I(1), I(2) and
        %       I(3) equivalent to the percentage error (0.1 = 10%) of the 
        %       moments of inertia around x, y and z body axis, respectively; or
        %       - Symmetric matrix of size 3x3, with I(1,1), I(2,2) and
        %       I(3,3) equivalent to the percentage error of the moments 
        %       of inertia around x, y and z body axis, respectively. 
        %       I may not necessarily be diagonal.
        %       I(i) or I(i,j) may be either positive or negative, where
        %       -0.1 is equivalent to -10% of error in relation to the
        %       inertia value of element i or (i,j) set by setInertia().
        %
        %   See also SETMASS, SETFRICTION, ADDROTOR.
        
            if isnumeric(inertiaError) && (isequal(size(inertiaError),[1 3]) || isequal(size(inertiaError),[3 1]) || (isequal(size(inertiaError),[3 3]) ))
                if ~isequal(size(inertiaError),[3 3])
                    obj.inertiaTensorPError_ = [inertiaError(1) 0 0; 0 inertiaError(2) 0; 0 0 inertiaError(3)];
                else
                    obj.inertiaTensorPError_ = inertiaError;
                end
                if obj.verbose_ == true
                    disp('Inertia tensor error set to (%):')
                    disp(100.*obj.inertiaTensorPError_)
                end
            else
                error('Inertia percentage error matrix must be numeric, have at least 3 values and be symmetric in case it is a tensor!')
            end
        end         
        function setFrictionError(obj, frictionError)    
        %SETFRICTIONERROR  Sets the percentage error for the translational body drag coefficients.
        %
        %   SETFRICTIONERROR(A) Sets the percentage error for the translational 
        %   body aerodynamic drag coefficients for multicopter object.
        %   A must be numeric, related to the body reference frame and:
        %       - Column or row vector of length 3, with A(1), A(2) and
        %       A(3) equivalent to the percentage error for the drag 
        %       coefficients in x, y and z body axis directions, respectively; or
        %       - Matrix of size 3x3, with A(1,1), A(2,2) and
        %       A(3,3) equivalent to the percentage error for the drag 
        %       coefficients in x, y and z body axis directions, respectively. 
        %       Other elements of A may not necessarily be zero.
        %       A(i,j) = 0.1 represents a 10% error in relation to the drag
        %       coefficient of the element (i,j) of the translational
        %       friction matrix set using setFriction(). A(i,j) may be
        %       either positive or negative.
        %
        %   See also SETMASS, SETINERTIA, ADDROTOR.
        
            if isnumeric(frictionError) && (isequal(size(frictionError),[1 3]) || isequal(size(frictionError),[3 1]) || isequal(size(frictionError),[3 3]))
                if ~isequal(size(frictionError),[3 3])
                    obj.translationalFrictionPError_ = [frictionError(1) 0 0; 0 frictionError(2) 0; 0 0 frictionError(3)];
                else
                    obj.translationalFrictionPError_ = frictionError;
                end
                if obj.verbose_ == true
                    disp('Translational body drag coefficients error set to (%):')
                    disp(100.*obj.translationalFriction_)
                end
            else
                error('The translational percentage error friction matrix must be numeric and have at least 3 values!');
            end
        end
        function setCGPositionError(obj, error)
        %SETCGPOSITIONERROR  Configures the position error for the center of gravity.
        %
        %   SETCGPOSITIONERROR(P) Sets a new position for the center of
        %   gravity (CG) in relation to the original (nominal) position
        %   located at the origin of the body reference frame.
        %   P must be a numeric array of length 3, specifying the relative
        %   position of the new CG in relation to the body reference frame.
        %
        %   See also SETMASS, SET INERTIA, ADDROTOR, SETROTORPOSITION.
        
            if isnumeric(error) && length(error)==3
                obj.cgPositionError_ = error;
                if isequal(size(obj.cgPositionError_),[1 3])
                    obj.cgPositionError_ = obj.cgPositionError_';
                end
                if obj.verbose_ == true
                    disp(['CG error set to: ',num2str(reshape(obj.cgPositionError_,[1 3]))])
                end
            else
                error('CG position error must be a numeric vector of length 3.')
            end
        end       
        function setPayload(obj, relativePosition, mass, inertiaTensor)
        %UNTITLED Summary of this function goes here
        %   Detailed explanation goes here
            if ~isempty(obj.mass_) && ~isempty(obj.inertiaTensor_)
                if isnumeric(relativePosition) && isvector(relativePosition) && ...
                        isnumeric(mass) && isnumeric(inertiaTensor) && isequal(size(inertiaTensor),[3 3])
                    Rcm = mass*relativePosition/(obj.mass_+mass);
                    obj.setCGPositionError(Rcm);
                    obj.setMassError(mass/obj.mass_);
                    
                    
                    relativeInertia(1,1) = obj.inertiaTensor_(1,1) + obj.mass_*(Rcm(2)^2+Rcm(3)^2);
                    relativeInertia(2,2) = obj.inertiaTensor_(2,2) + obj.mass_*(Rcm(1)^2+Rcm(3)^2);   
                    relativeInertia(3,3) = obj.inertiaTensor_(3,3) + obj.mass_*(Rcm(1)^2+Rcm(2)^2); 
                    relativeInertia(1,2) = obj.inertiaTensor_(1,2) + obj.mass_*(Rcm(1)*Rcm(2));
                    relativeInertia(2,3) = obj.inertiaTensor_(2,3) + obj.mass_*(Rcm(2)*Rcm(3));
                    relativeInertia(3,1) = obj.inertiaTensor_(3,1) + obj.mass_*(Rcm(3)*Rcm(1));
                    relativeInertia(2,1) = obj.inertiaTensor_(2,1) + obj.mass_*(Rcm(1)*Rcm(2));
                    relativeInertia(3,1) = obj.inertiaTensor_(3,2) + obj.mass_*(Rcm(2)*Rcm(3));
                    relativeInertia(1,3) = obj.inertiaTensor_(1,3) + obj.mass_*(Rcm(3)*Rcm(1));
                    
                    Rpl = relativePosition-Rcm;
                    plInertia(1,1) = inertiaTensor(1,1) + mass*(Rpl(2)^2+Rpl(3)^2);
                    plInertia(2,2) = inertiaTensor(2,2) + mass*(Rpl(1)^2+Rpl(3)^2);   
                    plInertia(3,3) = inertiaTensor(3,3) + mass*(Rpl(1)^2+Rpl(2)^2); 
                    plInertia(1,2) = inertiaTensor(1,2) + mass*(Rpl(1)*Rpl(2));
                    plInertia(2,3) = inertiaTensor(2,3) + mass*(Rpl(2)*Rpl(3));
                    plInertia(3,1) = inertiaTensor(3,1) + mass*(Rpl(3)*Rpl(1));
                    plInertia(2,1) = inertiaTensor(2,1) + mass*(Rpl(1)*Rpl(2));
                    plInertia(3,1) = inertiaTensor(3,2) + mass*(Rpl(2)*Rpl(3));
                    plInertia(1,3) = inertiaTensor(1,3) + mass*(Rpl(3)*Rpl(1));
                    
                    newInertia = relativeInertia+plInertia;
                    inertiaError = (newInertia-obj.inertiaTensor_)./obj.inertiaTensor_;
                    inertiaError(isnan(inertiaError)) = 0;
                    inertiaError(isinf(inertiaError)) = 0;
                    
                    obj.setInertiaError(inertiaError);
                    if obj.verbose_ == true
                        disp('Payload configured.')
                    end
                else
                    error('Payload relative position must be numeric and a vector. Payload mass must be numeric. Payload inertia tensor must be numeric and of size 3x3.')
                end
            else
                error('Configure mass and inertia tensor before adding payload.')
            end
        end
                        
        % Rotor related
        function rotorID = addRotor(obj, varargin)    
        %ADDROTOR  Adds rotor to multicopter object.
        %
        %   rotorID = ADDROTOR() Adds a rotor to multicopter and returns
        %   the respective rotor ID, which is used for further rotor
        %   configuration. All rotor settings are created empty and rotor
        %   is set ro 'running'.
        %
        %   rotorID = ADDROTOR(P) does the same as ADDROTOR(), but
        %   configures rotor at position P in relation to body reference
        %   frame. P must be a numeric column vector of length 3.
        %
        %   rotorID = ADDROTOR(P,O) does the same as ADDROTOR(P), but
        %   configures rotor at orientation O in relation to body reference
        %   frame. O must be a numeric column vector of length 3.
        %
        %   rotorID = ADDROTOR(P,O,I) does the same as ADDROTOR(P,O), but
        %   sets rotor inertia around its rotation axis to I. I must be
        %   numeric scalar.
        %
        %   rotorID = ADDROTOR(P,O,I,L) does the same as ADDROTOR(P,O,I), 
        %   but sets rotor lift coefficient to L. L must be numeric scalar.
        %
        %   rotorID = ADDROTOR(P,O,I,L,D) does the same as ADDROTOR(P,O,I,L), 
        %   but sets rotor drag coefficient to D. D must be numeric scalar.
        %
        %   rotorID = ADDROTOR(P,O,I,L,D,M) does the same as ADDROTOR(P,O,I,L,D), 
        %   but sets rotor maximum speed to M. M may be numeric scalar or infinity.
        %   Maximum speed is set to infinity in case M is not specified.
        %
        %   rotorID = ADDROTOR(P,O,I,L,D,M,m) does the same as ADDROTOR(P,O,I,L,D,M), 
        %   but sets rotor minimum speed to m. m may be numeric scalar.
        %   Minimum speed is set to zero in case m is not specified.
        %
        %   rotorID = ADDROTOR(P,O,I,L,D,M,m,T) does the same as ADDROTOR(P,O,I,L,D,M,m), 
        %   but sets the rotor transfer function coeficients as T. T = [a;b;c],
        %   where a, b, c make up the following controlled motor TF:
        %   G(s) = a/(s^2+b*s+c).
        %   T is set so to obtain 500 rad/s and 0.75 for natural frequency
        %   and damping ratio, respectively, in case T is not specified.
        %
        %   rotorID = ADDROTOR(P,O,I,L,D,M,m,T,A) does the same as ADDROTOR(P,O,I,L,D,M,m,T), 
        %   but sets the rotor maximum acceleration rate to A.
        %
        %   ADDROTOR can be used during simulation, but in most cases it is
        %   suggested to add all rotors during initial configuration and set
        %   them as fail or running acordingly. 
        %
        %   Adding a rotor during simulation will make the log matrix to
        %   change size and waste memory, but keeping track of
        %   modifications.
        %
        %   See also REMOVEROTOR, SETROTOR, SETROTORRUNNING , SETROTORSTATUS,
        %   SETROTORPOSITION, SETROTORORIENTATION, SETROTORINERTIA, 
        %   SETROTORLIFTCOEFF, SETROTORDRAGCOEFF, SETROTORMAXSPEED.
        
            rotorID = obj.numberOfRotors_+1;
            obj.numberOfRotors_ = rotorID;
            
            obj.rotor_(rotorID).position = [];
            obj.rotor_(rotorID).orientation = [];
            obj.rotor_(rotorID).inertia = [];
            obj.rotor_(rotorID).liftCoeff.data = [];
            obj.rotor_(rotorID).dragCoeff.data = [];
            obj.rotor_(rotorID).liftCoeff.fit = [];
            obj.rotor_(rotorID).dragCoeff.fit = [];
            obj.rotor_(rotorID).status = {'free','responding','motor ok','prop ok'};
            obj.rotor_(rotorID).motorEfficiency = 1;
            obj.rotor_(rotorID).propEfficiency = 1;
            obj.rotor_(rotorID).inputSetPoint = [];
            obj.rotor_(rotorID).maxSpeed = inf;
            obj.rotor_(rotorID).minSpeed = 0;
            obj.rotor_(rotorID).transferFunction = [250000;750;250000];
            obj.rotor_(rotorID).Rm = 0.0975;
            obj.rotor_(rotorID).Kt = 0.02498;
            obj.rotor_(rotorID).Kv = 340;
            obj.rotor_(rotorID).Io = 0.6/10;
            obj.rotor_(rotorID).maxVoltage = 22;
            obj.initialState_.input(rotorID) = 0;
            obj.initialState_.rotor(rotorID).speed = 0;
            obj.initialState_.rotor(rotorID).acceleration = 0;

            obj.rotor_(rotorID).inertiaPError = 0;
            obj.rotor_(rotorID).liftCoeffPError = 0;
            obj.rotor_(rotorID).dragCoeffPError = 0;
            obj.rotor_(rotorID).transferFunctionPError = [0;0;0];   
            
            obj.rotor_(rotorID).stuckTransitionPeriod = [];
            
            obj.previousState_.input(rotorID) = 0;
            obj.previousState_.rotor(rotorID).speed = 0;
            obj.previousState_.rotor(rotorID).acceleration = 0;
            
            if obj.verbose_ == true
                disp(['Added rotor ',num2str(rotorID),' to aircraft.'])
            end
            obj.setRotor(rotorID, varargin);
        end        
        function setRotor(obj, rotorID, varargin)
        %SETROTOR  Configures specified rotor.
        %
        %   SETROTOR(ID) Does nothing.
        %
        %   SETROTOR(rotorID,P) Configures rotor at position P in relation
        %   to body reference frame. rotorID may be a scalar or a vector,
        %   in arbitrary order, specifying which rotors will be configured.
        %   P must be a numeric array of size 3xN, where each column vector
        %   is a position vector in relation to body reference frame and 
        %   N = length(rotorID).
        %   Each column vector relates to the rotor ID specified at the
        %   same position of the rotorID vector. 
        %   
        %
        %   SETROTOR(rotorID,P,O) does the same as SETROTOR(rotorID,P), but
        %   configures rotors at orientation O in relation to body reference
        %   frame. O must be a numeric array of size 3xN.
        %
        %   SETROTOR(rotorID,P,O,I) does the same as SETROTOR(rotorID,P,O),
        %   but sets rotor inertia around its rotation axis to I. I may be
        %   a numeric scalar or a numeric vector the same length as
        %   rotorID.
        %
        %   SETROTOR(rotorID,P,O,I,L) does the same as 
        %   SETROTOR(rotorID,P,O,I), but sets rotor lift coefficient to L.
        %   L may be a numeric scalar or a numeric vector the same length
        %   as rotorID.
        %
        %   SETROTOR(rotorID,P,O,I,L,D) does the same as 
        %   SETROTOR(rotorID,P,O,I,L), but sets rotor drag coefficient to D.
        %   D mmay be a numeric scalar or a numeric vector the same length
        %   as rotorID.
        %
        %   SETROTOR(rotorID,P,O,I,L,D,M) does the same as 
        %   SETROTOR(rotorID,P,O,I,L,D), but sets rotor maximum allowed speed to M.
        %   M may be a numeric scalar (including infinity) or a numeric 
        %   vector the same length as rotorID.
        %
        %   SETROTOR(rotorID,P,O,I,L,D,M,m) does the same as SETROTOR(rotorID,P,O,I,L,D,M), 
        %   but sets rotor minimum speed to m. m may be numeric scalar.
        %   Minimum speed is set to zero in case m is not specified.
        %
        %   SETROTOR(rotorID,P,O,I,L,D,M,m,T) does the same as SETROTOR(rotorID,P,O,I,L,D,M,m), 
        %   but sets the rotor transfer function coeficients as T. T = [a,b,c],
        %   where a, b, c make up the following controlled motor TF:
        %   G(s) = a/(s^2+b*s+c).
        %   T must be a numeric array of size 3xN, where each column vector
        %   is a position vector in relation to body reference frame and 
        %   N = length(rotorID).
        %   T is set so to obtain 500 rad/s and 0.75 for natural frequency
        %   and damping ratio, respectively, in case T is not specified.
        %
        %   SETROTOR may be substituded by setting rotor position,
        %   orientation, inertia, lift and drag coefficients separately
        %   using functions presented below.
        %
        %   SETROTOR can be used during simulation, but parameters are
        %   considered constant and not dynamic variables, for this model 
        %   version. 
        %   Suggested use during simulation is for control robustness
        %   evaluation.
        %
        %   See also ADDROTOR, SETROTORPOSITION, SETROTORORIENTATION,
        %   SETROTORINERTIA, SETROTORLIFTCOEFF, SETROTORDRAGCOEFF, SETROTORMAXSPEED.
        
            if length(varargin)>=1 && ~isempty(varargin{1})
                obj.setRotorPosition(rotorID, varargin{1});
            end
            if length(varargin)>=2
                obj.setRotorOrientation(rotorID, varargin{2});
            end
            if length(varargin)>=3
                obj.setRotorInertia(rotorID, varargin{3});
            end
            if length(varargin)>=4
                obj.setRotorLiftCoeff(rotorID, varargin{4});
            end
            if length(varargin)>=5
                obj.setRotorDragCoeff(rotorID, varargin{5});
            end
            if length(varargin)>=6
                obj.setRotorMaxSpeed(rotorID, varargin{6});
            end
            if length(varargin)>=7
                obj.setRotorMinSpeed(rotorID, varargin{7});
            end
            if length(varargin)>=8
                obj.setRotorTF(rotorID, varargin{8});
            end
        end  
        function setRotorPosition(obj, rotorID, position)
        %SETROTORPOSITION  Configures position of specified rotor.
        %
        %   SETROTORPOSITION(rotorID,P) Configures rotor at position P in 
        %   relation to body reference frame. rotorID may be a scalar or a 
        %   vector, in arbitrary order, specifying which rotors will be 
        %   configured.
        %   P must be a numeric array of size 3xN, where each column vector
        %   is a position vector in relation to body reference frame and 
        %   N = length(rotorID).
        %   Each column vector relates to the rotor ID specified at the
        %   same position of the rotorID vector. 
        %
        %   SETROTORPOSITION can be used during simulation, but parameters 
        %   are considered constant and not dynamic variables, for this model 
        %   version. 
        %   Suggested use during simulation is for control robustness
        %   evaluation.
        %
        %   See also ADDROTOR, SETROTOR, SETROTORORIENTATION,
        %   SETROTORINERTIA, SETROTORLIFTCOEFF, SETROTORDRAGCOEFF, SETROTORMAXSPEED.
        
            if iscell(position)
                warning('Input will be converted to numeric array.')
                position = cell2mat(position);
            end
            if isnumeric(position) && isequal(size(position),[3 length(rotorID)]) && length(rotorID)<=obj.numberOfRotors_
                position = mat2cell(position,3,ones(1,size(position,2)));
                [obj.rotor_(rotorID).position] = position{:};
                if obj.verbose_ == true
                    for it=1:length(rotorID)
                        disp(['Rotor ',num2str(rotorID(it)),' position set to: ',num2str(reshape(position{it},[1 3])),' m.'])
                    end
                end
            else
                error('Rotor position must be an array of numeric column (3x1) vectors the same length as the vector of rotor IDs, which must not exceed the number of rotors in the multicopter!')
            end
        end       
        function setRotorOrientation(obj, rotorID, orientation)
        %SETROTORORIENTATION  Configures orientation of specified rotor.
        %
        %   SETROTORORIENTATION(rotorID,O) Configures rotor at orientation
        %   O in relation to body reference frame. rotorID may be a scalar 
        %   or a vector, in arbitrary order, specifying which rotors will 
        %   be configured.
        %   O must be a numeric array of size 3xN, where each column vector
        %   is an orientation vector in relation to body reference frame and 
        %   N = length(rotorID).
        %   Each column vector relates to the rotor ID specified at the
        %   same position of the rotorID vector.
        %
        %   SETROTORORIENTATION can be used during simulation, but 
        %   parameters are considered constant and not dynamic variables, 
        %   for this model version. 
        %   Suggested use during simulation is for control robustness
        %   evaluation.
        %
        %   See also ADDROTOR, SETROTORPOSITION, SETROTOR,
        %   SETROTORINERTIA, SETROTORLIFTCOEFF, SETROTORDRAGCOEFF, SETROTORMAXSPEED.
        
            if iscell(orientation)
                warning('Input will be converted to numeric array.')
                orientation = cell2mat(orientation);
            end
            if isnumeric(orientation) && isequal(size(orientation),[3 length(rotorID)]) && length(rotorID)<=obj.numberOfRotors_
                orientation = mat2cell(orientation,3,ones(1,size(orientation,2)));
                [obj.rotor_(rotorID).orientation] = orientation{:};
                if obj.verbose_ == true
                    for it=1:length(rotorID)
                        disp(['Rotor ',num2str(rotorID(it)),' orientation set to: ',num2str(reshape(orientation{it},[1 3])),' m.'])
                    end
                end
            else
                error('Rotor orientation must be an array of numeric column (3x1) vectors the same length as the vector of rotor IDs, which must not exceed the number of rotors in the multicopter!')
            end
        end                
        function setRotorInertia(obj, rotorID, inertia)
        %SETROTORINERTIA  Configures inertia for specified rotor.
        %
        %   SETROTORINERTIA(rotorID,I) Sets rotor inertia around its
        %   rotation axis to I. rotorID may be a scalar or a vector,
        %   in arbitrary order, specifying which rotors will be configured.
        %   I may be a numeric scalar or a numeric vector the same length as
        %   rotorID.
        %   Each index relates to the rotor ID specified at the
        %   same index position of the rotorID vector. 
        %   
        %   SETROTORINERTIA can be used during simulation, but parameters are
        %   considered constant and not dynamic variables, for this model 
        %   version. 
        %   Suggested use during simulation is for control robustness
        %   evaluation.
        %
        %   See also ADDROTOR, SETROTORPOSITION, SETROTORORIENTATION,
        %   SETROTOR, SETROTORLIFTCOEFF, SETROTORDRAGCOEFF,SETROTORMAXSPEED.
        
            if iscell(inertia)
                warning('Input will be converted to numeric array.')
                inertia = cell2mat(inertia);
            end
            if isvector(inertia) && isnumeric(inertia) && (length(inertia)==length(rotorID)) && length(rotorID)<=obj.numberOfRotors_
                inertia = num2cell(inertia);
                [obj.rotor_(rotorID).inertia] = inertia{:};
                if obj.verbose_ == true
                    for it=1:length(rotorID)
                        disp(['Rotor ',num2str(rotorID(it)),' inertia set to: ',num2str(inertia{it}),' kg.m^2.'])
                    end
                end
            else
                error('Rotor inertia coefficients must be a vector of numeric values the same size as rotorID, which must not exceed the number of rotors in the multicopter!')
            end
        end        
        function setRotorLiftCoeff(obj, rotorID, liftCoeff, varargin)
        %SETROTORLIFTCOEFF  Configures lift coefficient for specified rotor.
        %
        %   SETROTORLIFTCOEFF(rotorID,L) UPDATE DESCRIPTION Sets rotor lift coefficient to L.
        %   rotorID may be a scalar or a vector, in arbitrary order, 
        %   specifying which rotors will be configured.
        %   L may be a numeric scalar or a numeric vector the same length
        %   as rotorID.
        %   Each vector position relates to the rotor ID specified at the
        %   same position of the rotorID vector.
        %
        %   SETROTORLIFTCOEFF can be used during simulation, but parameters are
        %   considered constant and not dynamic variables, for this model 
        %   version. 
        %   Suggested use during simulation is for control robustness
        %   evaluation.
        %
        %   See also ADDROTOR, SETROTORPOSITION, SETROTORORIENTATION,
        %   SETROTORINERTIA, SETROTOR, SETROTORDRAGCOEFF, SETROTORMAXSPEED.
        
            if iscell(liftCoeff)
                warning('Input will be converted to numeric array.')
                liftCoeff = cell2mat(liftCoeff);
            end
            if isnumeric(liftCoeff)
                if isvector(rotorID) && isscalar(liftCoeff)
                    for it=1:length(rotorID)
                        obj.rotor_(rotorID(it)).liftCoeff.data = liftCoeff;
                        obj.rotor_(rotorID(it)).liftCoeff.fit = [];
                    end                    
                else
                    if isvector(rotorID) && isequal(size(liftCoeff),size(rotorID))
                        for it=1:length(rotorID)
                            obj.rotor_(rotorID(it)).liftCoeff.data = liftCoeff(it);
                            obj.rotor_(rotorID(it)).liftCoeff.fit = [];
                        end  
                    else
                        if isvector(rotorID) && size(liftCoeff,2)==2 && ischar(varargin{1})
                            for it=1:length(rotorID)
                                obj.rotor_(rotorID(it)).liftCoeff.data = liftCoeff;
                                obj.rotor_(rotorID(it)).liftCoeff.fit = fit(liftCoeff(:,1),liftCoeff(:,2),varargin{1});
                            end  
                        else
                            error('Wrong input. Read function help')
                        end
                    end
                end
            else
                error('Lift coefficients must be numeric values.')
            end
        end        
        function setRotorDragCoeff(obj, rotorID, dragCoeff, varargin)
        %SETROTORDRAGCOEFF  Configures lift coefficient for specified rotor.
        %
        %   SETROTORDRAGCOEFF(rotorID,D) UPDATE DESCRIPTION Sets rotor drag coefficient to D.
        %   rotorID may be a scalar or a vector, in arbitrary order, 
        %   specifying which rotors will be configured.
        %   D may be a numeric scalar or a numeric vector the same length
        %   as rotorID.
        %   Each vector position relates to the rotor ID specified at the
        %   same position of the rotorID vector.
        %
        %   SETROTORDRAGCOEFF can be used during simulation, but parameters are
        %   considered constant and not dynamic variables, for this model 
        %   version. 
        %   Suggested use during simulation is for control robustness
        %   evaluation.
        %
        %   See also ADDROTOR, SETROTORPOSITION, SETROTORORIENTATION,
        %   SETROTORINERTIA, SETROTORLIFTCOEFF, SETROTOR, SETROTORMAXSPEED.
        
            if iscell(dragCoeff)
                warning('Input will be converted to numeric array.')
                dragCoeff = cell2mat(dragCoeff);
            end
            if isnumeric(dragCoeff)
                if isvector(rotorID) && isscalar(dragCoeff)
                    for it=1:length(rotorID)
                        obj.rotor_(rotorID(it)).dragCoeff.data = dragCoeff;
                        obj.rotor_(rotorID(it)).dragCoeff.fit = [];
                    end                    
                else
                    if isvector(rotorID) && isequal(size(dragCoeff),size(rotorID))
                        for it=1:length(rotorID)
                            obj.rotor_(rotorID(it)).dragCoeff.data = dragCoeff(it);
                            obj.rotor_(rotorID(it)).dragCoeff.fit = [];
                        end  
                    else
                        if isvector(rotorID) && size(dragCoeff,2)==2 && ischar(varargin{1})
                            for it=1:length(rotorID)
                                obj.rotor_(rotorID(it)).dragCoeff.data = dragCoeff;
                                obj.rotor_(rotorID(it)).dragCoeff.fit = fit(dragCoeff(:,1),dragCoeff(:,2),varargin{1});
                            end  
                        else
                            error('Wrong input. Read function help')
                        end
                    end
                end
            else
                error('Drag coefficients must be numeric values.')
            end
        end
        function setRotorMaxSpeed(obj, rotorID, maxSpeed)
        %SETROTORMAXSPEED  Configures maximum speed (rad/s) for specified rotor.
        %
        %   SETROTORMAXSPEED(rotorID,M) Sets rotor maximum allowed speed to M.
        %   rotorID may be a scalar or a vector, in arbitrary order, 
        %   specifying which rotors will be configured.
        %   M may be a numeric scalar (including infinity) or a numeric 
        %   vector the same length as rotorID.
        %   Each vector position relates to the rotor ID specified at the
        %   same position of the rotorID vector.
        %
        %   SETROTORMAXSPEED can be used during simulation.
        %
        %   See also ADDROTOR, SETROTORPOSITION, SETROTORORIENTATION,
        %   SETROTORINERTIA, SETROTORLIFTCOEFF, SETROTORDRAGCOEFFICIENT, 
        %   SETROTOR.
        
            if iscell(maxSpeed)
                warning('Input will be converted to numeric array.')
                maxSpeed = cell2mat(maxSpeed);
            end
            if isvector(maxSpeed) && isnumeric(maxSpeed) && (length(maxSpeed)==length(rotorID)) && length(rotorID)<=obj.numberOfRotors_
                maxSpeed = num2cell(maxSpeed);
                [obj.rotor_(rotorID).maxSpeed] = maxSpeed{:};
            else
                error('Maximum rotor speed must be a vector of numeric values the same size as rotorID, which must not exceed the number of rotors in the multicopter!')
            end
        end
        function setRotorMinSpeed(obj, rotorID, minSpeed)
        %SETROTORMINSPEED  Configures minimum speed (rad/s) for specified rotor.
        %
        %   SETROTORMINSPEED(rotorID,M) Sets rotor minimum allowed speed to M.
        %   rotorID may be a scalar or a vector, in arbitrary order, 
        %   specifying which rotors will be configured.
        %   M may be a numeric scalar or a numeric 
        %   vector the same length as rotorID.
        %   Each vector position relates to the rotor ID specified at the
        %   same position of the rotorID vector.
        %
        %   SETROTORMINSPEED can be used during simulation.
        %
        %   See also ADDROTOR, SETROTORPOSITION, SETROTORORIENTATION,
        %   SETROTORINERTIA, SETROTORLIFTCOEFF, SETROTORDRAGCOEFFICIENT, 
        %   SETROTOR.
        
            if iscell(minSpeed)
                warning('Input will be converted to numeric array.')
                minSpeed = cell2mat(minSpeed);
            end
            if isvector(minSpeed) && isnumeric(minSpeed) && (length(minSpeed)==length(rotorID)) && length(rotorID)<=obj.numberOfRotors_
                minSpeed = num2cell(minSpeed);
                [obj.rotor_(rotorID).minSpeed] = minSpeed{:};
            else
                error('Minimum rotor speed must be a vector of numeric values the same size as rotorID, which must not exceed the number of rotors in the multicopter!')
            end
        end
        function setRotorTF(obj, rotorID, TF)
        %SETROTORTF  Configures transfer function for specified rotor.
        %
        %   SETROTORTF(rotorID,TF) Configures rotor transfer function to be
        %   speed(rotorID) = input(rotorID)*a/(s^2+b*s+c), where T(:,rotorID) = [a;b;c]. 
        %   rotorID may be a scalar or a vector, in arbitrary order, 
        %   specifying which rotors will be configured.
        %   TF must be a numeric array of size 3xN, where each column vector
        %   is a tf coefficients vector according to relation above.
        %   Each column vector relates to the rotor ID specified at the
        %   same position of the rotorID vector. 
        %
        %   SETROTORTF can be used during simulation, but parameters 
        %   are considered constant and not dynamic variables, for this model 
        %   version. 
        %   Suggested use during simulation is for control robustness
        %   evaluation.
        %
        %   See also ADDROTOR, SETROTOR, SETROTORORIENTATION,
        %   SETROTORINERTIA, SETROTORLIFTCOEFF, SETROTORDRAGCOEFF, SETROTORMAXSPEED.
        
            if iscell(TF)
                warning('Input will be converted to numeric array.')
                TF = cell2mat(TF);
            end
            if isnumeric(TF) && isequal(size(TF),[3 length(rotorID)]) && length(rotorID)<=obj.numberOfRotors_
                TF = mat2cell(TF,3,ones(1,size(TF,2)));
                [obj.rotor_(rotorID).transferFunction] = TF{:};
            else
                error('Rotor TF must be an array of numeric column (3x1) vectors the same length as the vector of rotor IDs, which must not exceed the number of rotors in the multicopter!')
            end
        end  
        function setRotorRm(obj, rotorID, Rm)
        %SETROTORRM  Configures winding motor resistance (Ohms) for specified rotor.
        %
        %   SETROTORRM(rotorID,Rm) Sets winding resistance to Rm.
        %   rotorID may be a scalar or a vector, in arbitrary order, 
        %   specifying which rotors will be configured.
        %   Rm may be a numeric scalar or a numeric 
        %   vector the same length as rotorID of positive real numbers.
        %   Each vector position relates to the rotor ID specified at the
        %   same position of the rotorID vector.
        %
        %   SETROTORRM can be used during simulation.
        %
        %   See also ADDROTOR, SETROTORPOSITION, SETROTORORIENTATION,
        %   SETROTORINERTIA, SETROTORLIFTCOEFF, SETROTORDRAGCOEFFICIENT, 
        %   SETROTOR.
        
            if iscell(Rm)
                warning('Input will be converted to numeric array.')
                Rm = cell2mat(Rm);
            end
            if isvector(Rm) && isnumeric(Rm) && (length(Rm)==length(rotorID)) && length(rotorID)<=obj.numberOfRotors_ && all(Rm>=0)
                Rm = num2cell(Rm);
                [obj.rotor_(rotorID).Rm] = Rm{:};
            else
                error('Motor winding resistance must be a vector of positive numeric values the same size as rotorID, which must not exceed the number of rotors in the multicopter!')
            end
        end
        function setRotorKt(obj, rotorID, Kt)
        %SETROTORKT  Configures motor torque constant (N.m/A) for specified rotor.
        %
        %   SETROTORKT(rotorID,Kt) Sets torque constant to Kt.
        %   rotorID may be a scalar or a vector, in arbitrary order, 
        %   specifying which rotors will be configured.
        %   Kt may be a positive numeric scalar or a numeric 
        %   vector the same length as rotorID of positive real numbers.
        %   Each vector position relates to the rotor ID specified at the
        %   same position of the rotorID vector.
        %
        %   SETROTORKT can be used during simulation.
        %
        %   See also ADDROTOR, SETROTORPOSITION, SETROTORORIENTATION,
        %   SETROTORINERTIA, SETROTORLIFTCOEFF, SETROTORDRAGCOEFFICIENT, 
        %   SETROTOR.
        
            if iscell(Kt)
                warning('Input will be converted to numeric array.')
                Kt = cell2mat(Kt);
            end
            if isvector(Kt) && isnumeric(Kt) && (length(Kt)==length(rotorID)) && length(rotorID)<=obj.numberOfRotors_ && all(Kt>=0)
                Kt = num2cell(Kt);
                [obj.rotor_(rotorID).Kt] = Kt{:};
            else
                error('Motor torque constant must be a vector of positive numeric values the same size as rotorID, which must not exceed the number of rotors in the multicopter!')
            end
        end  
        function setRotorKv(obj, rotorID, Kv)
        %SETROTORKV  Configures motor speed constant (RPM/V) for specified rotor.
        %
        %   SETROTORKV(rotorID,Kv) Sets speed constant to Kv.
        %   rotorID may be a scalar or a vector, in arbitrary order, 
        %   specifying which rotors will be configured.
        %   Kv may be a positive numeric scalar or a numeric 
        %   vector the same length as rotorID of positive real numbers.
        %   Each vector position relates to the rotor ID specified at the
        %   same position of the rotorID vector.
        %
        %   SETROTORKV can be used during simulation.
        %
        %   See also ADDROTOR, SETROTORPOSITION, SETROTORORIENTATION,
        %   SETROTORINERTIA, SETROTORLIFTCOEFF, SETROTORDRAGCOEFFICIENT, 
        %   SETROTOR.
        
            if iscell(Kv)
                warning('Input will be converted to numeric array.')
                Kv = cell2mat(Kv);
            end
            if isvector(Kv) && isnumeric(Kv) && (length(Kv)==length(rotorID)) && length(rotorID)<=obj.numberOfRotors_ && all(Kv>=0)
                Kv = num2cell(Kv);
                [obj.rotor_(rotorID).Kv] = Kv{:};
            else
                error('Motor speed constant must be a vector of positive numeric values the same size as rotorID, which must not exceed the number of rotors in the multicopter!')
            end
        end 
        function setRotorIo(obj, rotorID, Io)
        %SETROTORIO  Configures motor idle current rate (A/V) for specified rotor.
        %
        %   SETROTORIO(rotorID,Io) Sets idle (no-load) current rate to Io.
        %   rotorID may be a scalar or a vector, in arbitrary order, 
        %   specifying which rotors will be configured.
        %   Io may be a positive numeric scalar or a numeric 
        %   vector the same length as rotorID of positive real numbers.
        %   Each vector position relates to the rotor ID specified at the
        %   same position of the rotorID vector.
        %
        %   SETROTORIO can be used during simulation.
        %
        %   See also ADDROTOR, SETROTORPOSITION, SETROTORORIENTATION,
        %   SETROTORINERTIA, SETROTORLIFTCOEFF, SETROTORDRAGCOEFFICIENT, 
        %   SETROTOR.
        
            if iscell(Io)
                warning('Input will be converted to numeric array.')
                Io = cell2mat(Io);
            end
            if isvector(Io) && isnumeric(Io) && (length(Io)==length(rotorID)) && length(rotorID)<=obj.numberOfRotors_ && all(Io>=0)
                Io = num2cell(Io);
                [obj.rotor_(rotorID).Io] = Io{:};
            else
                error('Motor idle current rate must be a vector of positive numeric values the same size as rotorID, which must not exceed the number of rotors in the multicopter!')
            end
        end 
        function setRotorMaxVoltage(obj, rotorID, maxVoltage)
        %SETROTORMAXVOLTAGE  Configures maximum voltage allowed on specified rotor.
        %
        %   SETROTORMAXVOLTAGE(rotorID,V) Sets maximum voltage to V.
        %   rotorID may be a scalar or a vector, in arbitrary order, 
        %   specifying which rotors will be configured.
        %   V may be a positive numeric scalar or a numeric 
        %   vector the same length as rotorID of positive real numbers.
        %   Each vector position relates to the rotor ID specified at the
        %   same position of the rotorID vector.
        %
        %   SETROTORMAXVOLTAGE can be used during simulation.
        %
        %   See also ADDROTOR, SETROTORPOSITION, SETROTORORIENTATION,
        %   SETROTORINERTIA, SETROTORLIFTCOEFF, SETROTORDRAGCOEFFICIENT, 
        %   SETROTOR.
        
            if iscell(maxVoltage)
                warning('Input will be converted to numeric array.')
                maxVoltage = cell2mat(maxVoltage);
            end
            if isvector(maxVoltage) && isnumeric(maxVoltage) && (length(maxVoltage)==length(rotorID)) && length(rotorID)<=obj.numberOfRotors_ && all(maxVoltage>=0)
                maxVoltage = num2cell(maxVoltage);
                [obj.rotor_(rotorID).maxVoltage] = maxVoltage{:};
            else
                error('Motor maximum voltage must be a vector of positive numeric values the same size as rotorID, which must not exceed the number of rotors in the multicopter!')
            end
        end 
        
        function setRotorInertiaError(obj, rotorID, error)
        %SETROTORINERTIAERROR  Configures percentage error for rotor inertia.
        %
        %   SETROTORINERTIAERROR(rotorID,I) Sets rotor inertia percentage error
        %   in relation to the nominal value set using setRotorInertia().
        %   rotorID may be a scalar or a vector in arbitrary order, specifying 
        %   which rotors will be configured.
        %   I may be a numeric scalar or a numeric vector the same length as
        %   rotorID. The elements of I may be either positive or negative,
        %   where 0.1 represents 10% of increase in the nominal value, for instance.
        %   Each index relates to the rotor ID specified at the
        %   same index position of the rotorID vector. 
        %
        %   See also ADDROTOR, SETROTORPOSITION, SETROTORORIENTATION,
        %   SETROTOR, SETROTORLIFTCOEFF, SETROTORDRAGCOEFF,SETROTORMAXSPEED.
        
            if iscell(error)
                warning('Input will be converted to numeric array.')
                error = cell2mat(error);
            end
            if isvector(error) && isnumeric(error) && (length(error)==length(rotorID)) && length(rotorID)<=obj.numberOfRotors_
                error = num2cell(error);
                [obj.rotor_(rotorID).inertiaPError] = error{:};
            else
                error('Rotor inertia percentage error must be a vector of numeric values the same size as rotorID, which must not exceed the number of rotors in the multicopter!')
            end
        end    
        function setRotorLiftCoeffError(obj, rotorID, error)
        %SETROTORLIFTCOEFFERROR  Configures percentage error for the rotor lift coefficient.
        %
        %   SETROTORLIFTCOEFFERROR(rotorID,L) Sets the percentage error for the
        %   rotor lift coefficient to L.
        %   rotorID may be a scalar or a vector, in arbitrary order, 
        %   specifying which rotors will be configured.
        %   L may be a numeric scalar or a numeric vector the same length
        %   as rotorID. The elements of L may be either positive or
        %   negative where 0.1 represents a 10% increase in the lift
        %   coefficient in relation to the nominal value set using
        %   setRotorLiftCoeff(), while -0.1 represents a 10% decrease.
        %   Each vector position relates to the rotor ID specified at the
        %   same position of the rotorID vector.
        %
        %   See also ADDROTOR, SETROTORPOSITION, SETROTORORIENTATION,
        %   SETROTORINERTIA, SETROTOR, SETROTORDRAGCOEFF, SETROTORMAXSPEED.
        
            if iscell(error)
                warning('Input will be converted to numeric array.')
                error = cell2mat(error);
            end
            if isvector(error) && isnumeric(error) && (length(error)==length(rotorID)) && length(rotorID)<=obj.numberOfRotors_
                error = num2cell(error);
                [obj.rotor_(rotorID).liftCoeffPError] = error{:};
            else
                error('Lift coefficients percentage error must be a vector of numeric values the same size as rotorID, which must not exceed the number of rotors in the multicopter!')
            end
        end        
        function setRotorDragCoeffError(obj, rotorID, error)
        %SETROTORDRAGCOEFFERROR  Configures the percentage error for the rotor lift coefficient.
        %
        %   SETROTORDRAGCOEFFERROR(rotorID,D) Sets the percentage error for the
        %   rotor drag coefficient to D.
        %   rotorID may be a scalar or a vector, in arbitrary order, 
        %   specifying which rotors will be configured.
        %   D may be a numeric scalar or a numeric vector the same length
        %   as rotorID. The elements of D may be either positive or
        %   negative where 0.1 represents a 10% increase in the lift
        %   coefficient in relation to the nominal value set using
        %   setRotorDragCoeff(), while -0.1 represents a 10% decrease.
        %   Each vector position relates to the rotor ID specified at the
        %   same position of the rotorID vector.
        %
        %   See also ADDROTOR, SETROTORPOSITION, SETROTORORIENTATION,
        %   SETROTORINERTIA, SETROTORLIFTCOEFF, SETROTOR, SETROTORMAXSPEED.
        
            if iscell(error)
                warning('Input will be converted to numeric array.')
                error = cell2mat(error);
            end
            if isvector(error) && isnumeric(error) && (length(error)==length(rotorID)) && length(rotorID)<=obj.numberOfRotors_
                error = num2cell(error);
                [obj.rotor_(rotorID).dragCoeffPError] = error{:};
            else
                error('Drag coefficients percentage error must be a vector of numeric values the same size as rotorID, which must not exceed the number of rotors in the multicopter!')
            end
        end
        function setRotorTFError(obj, rotorID, error)
        %SETROTORTFERROR  Configures transfer function percentage error for specified rotor.
        %
        %   SETROTORTFERROR(rotorID,TF) Sets the percentage error for the 
        %   rotor transfer function to TF, where TF(:,rotorID) = [a;b;c],
        %   are the percentage error for the transfer function coeficients
        %   in relation to nominal values set using setRotorTF().
        %   rotorID may be a scalar or a vector, in arbitrary order, 
        %   specifying which rotors will be configured.
        %   TF must be a numeric array of size 3xN, where each column vector
        %   is the percentage error for the transfer function of a rotor,
        %   according to relation above. ELements of TF may be either positive or
        %   negative where 0.1 represents a 10% increase in the coefficient in 
        %   relation to the nominal value, while -0.1 represents a 10% decrease.
        %   Each column vector relates to the rotor ID specified at the
        %   same position of the rotorID vector.
        %
        %   See also ADDROTOR, SETROTOR, SETROTORORIENTATION,
        %   SETROTORINERTIA, SETROTORLIFTCOEFF, SETROTORDRAGCOEFF, SETROTORMAXSPEED.
        
            if iscell(error)
                warning('Input will be converted to numeric array.')
                error = cell2mat(error);
            end
            if isnumeric(error) && isequal(size(error),[3 length(rotorID)]) && length(rotorID)<=obj.numberOfRotors_
                error = mat2cell(error,3,ones(1,size(error,2)));
                [obj.rotor_(rotorID).transferFunctionPError] = error{:};
            else
                error('Rotor TF percentage error must be an array of numeric column (3x1) vectors the same length as the vector of rotor IDs, which must not exceed the number of rotors in the multicopter!')
            end
        end
        function setRotorError(obj, rotorID, varargin)
        %SETROTORERROR  Configures errors for specified rotors in batch.
        %        
        %   SETROTORTFERROR(rotorID,I,L,D,TF) sets the rotor errors in
        %   batches where I,L,D and TF are the inertia, lift, drag and
        %   transfer function coefficients percentage error, respectively.
        %   I, L, D and TF are optional. 
        %
        %   SETROTORTFERROR(rotorID,I,L,D,TF) is equivalent to calling
        %   setRotorInertiaError(rotorID, I),
        %   setRotorLiftCoeffError(rotorID, L),
        %   setRotorDragCoeffError(rotorID, D), and
        %   setRotorTFError(rotorID, TF), in this order and if the argument
        %   is provided.
        %
        %   See also ADDROTOR, SETROTOR, SETROTORORIENTATION,
        %   SETROTORINERTIA, SETROTORLIFTCOEFF, SETROTORDRAGCOEFF, SETROTORMAXSPEED.
            if length(varargin)>=1 && ~isempty(varargin{1})
                obj.setRotorInertiaError(rotorID, varargin{1});
            end
            if length(varargin)>=2
                obj.setRotorLiftCoeffError(rotorID, varargin{2});
            end
            if length(varargin)>=3
                obj.setRotorDragCoeffError(rotorID, varargin{3});
            end
            if length(varargin)>=4
                obj.setRotorTFError(rotorID, varargin{4});
            end
        end 
        
        % Simulation related
        function setTimeStep(obj, timeStep)
        %SETTIMESTEP  Sets time step for simulation.
        %
        %   SETTIMESTEP(dt) Sets the time step used for simulation.
        %   Simulation outputs are calculated according to this value.
        %   dt must be scalar and greater than zero.
        %
        %   See also SETINITIALSTATE, SETINITIALPOSITION,
        %   SETINITIALATTITUDE, SETINITIALVELOCITY,
        %   SETINITIALANGULARVELOCITY, SETINITIALROTORSPEEDS,
        %   SETINITIALINPUT
        
            if isscalar(timeStep) && isnumeric(timeStep) && timeStep>0
                obj.timeStep_ = timeStep;
            else
                error('Simulation step period must be scalar and greater than zero');
            end
        end
        function setInitialState(obj, state)
        %SETINITIALSTATE  Sets initial states and inputs for simulation.
        %
        %   SETINITIALSTATE(A) Sets the initial states and inputs for
        %   simulation. A must be a numeric vector of length 13+2*nR, where
        %   nR is the number of rotors already added to this multicopter.
        %   
        %   A must be comprised of position, attitude, velocity, angular
        %   velocity, rotor speeds and inputs, in this order.
        %
        %   SETINITIALSTATE may be substituded by setting initial position,
        %   attitude, velocity, angular velocity, rotor speeds and inputs
        %   separately, using functions stated below.
        %
        %   The initial states are used for simulation at time 0. Every
        %   time the simulation is reset, the initial states are used.
        %
        %   See also SETTIMESTEP, SETINITIALPOSITION,
        %   SETINITIALATTITUDE, SETINITIALVELOCITY,
        %   SETINITIALANGULARVELOCITY, SETINITIALROTORSPEEDS,
        %   SETINITIALINPUT
        
            if isnumeric(state) && isvector(state) && length(state)==(13+2*obj.numberOfRotors_)
                obj.setInitialPosition(state(1:3))
                obj.setInitialAttitude(state(4:7))
                obj.setInitialVelocity(state(8:10))
                obj.setInitialAngularVelocity(state(11:13))
                if length(state)>13
                    obj.setInitialRotorSpeeds(state(14:(obj.numberOfRotors_+13)))
                    obj.setInitialInput(state((obj.numberOfRotors_+14):end))
                end
            else
                error('Initial state must be a numeric array vector comprised of position, attitude, velocity, angular velocity, rotor speeds and inputs in this order.')
            end
        end        
        function setInitialPosition(obj, position)
        %SETINITIALPOSITION  Sets initial multicopter position for simulation.
        %
        %   SETINITIALPOSITION(P) Sets the initial position of the
        %   multicopter for simulation. P must be a numeric vector of
        %   length 3.
        %
        %   The initial states are used for simulation at time 0. Every
        %   time the simulation is reset, the initial states are used.
        %
        %   See also SETTIMESTEP, SETINITIALSTATE,
        %   SETINITIALATTITUDE, SETINITIALVELOCITY,
        %   SETINITIALANGULARVELOCITY, SETINITIALROTORSPEEDS,
        %   SETINITIALINPUT
        
            if iscell(position)
                warning('Input will be converted to numeric array. Assure it can represent an array of length 3.')
                position = cell2mat(position);
            end
            if isnumeric(position) && isvector(position) && length(position)==3
                if isequal(size(position),[1 3])
                    position = position';
                end
                obj.initialState_.position = position;
            else
                error('Initial aircraft position must be a numeric vector of length 3!')
            end
        end 
        function setInitialAttitude(obj, attitude)
        %SETINITIALATTITUDE  Sets initial multicopter attitude for simulation.
        %
        %   SETINITIALATTITUDE(A) Sets the initial attitude of the
        %   multicopter in relation to inertial (absolute) frame, for simulation. 
        %   A must be a numeric vector of quaternions (length 4). 
        %   For euler angles, use setInitialAttitude(toQuaternion(A)), where A
        %   is a vector of euler angles of length 3.
        %
        %   The initial states are used for simulation at time 0. Every
        %   time the simulation is reset, the initial states are used.
        %
        %   See also TOQUATERNION, SETTIMESTEP, SETINITIALSTATE,
        %   SETINITIALPOSITION, SETINITIALVELOCITY,
        %   SETINITIALANGULARVELOCITY, SETINITIALROTORSPEEDS,
        %   SETINITIALINPUT
        
            if iscell(attitude)
                warning('Input will be converted to numeric array. Assure it can represent an array of length 4 (quaternion).')
                attitude = cell2mat(attitude);
            end
            if isnumeric(attitude) && isvector(attitude) && length(attitude)==4
                if isequal(size(attitude),[1 4])
                    attitude = attitude';
                end
                obj.initialState_.attitude = attitude;
            else
                error('Initial aircraft attitude must be a numeric vector of quaternions (length 4)! Tip.: use toQuaternion() to convert from Euler Angles')
            end
        end        
        function setInitialVelocity(obj, velocity)
        %SETINITIALVELOCITY  Sets initial multicopter velocity for simulation.
        %
        %   SETINITIALVELOCITY(V) Sets the initial velocity of the
        %   multicopter in relation to inertial (absolute) frame, for simulation. 
        %   V must be a numeric vector of length 3. 
        %
        %   The initial states are used for simulation at time 0. Every
        %   time the simulation is reset, the initial states are used.
        %
        %   See also SETTIMESTEP, SETINITIALSTATE,
        %   SETINITIALPOSITION, SETINITIALATTITUDE,
        %   SETINITIALANGULARVELOCITY, SETINITIALROTORSPEEDS,
        %   SETINITIALINPUT
        
            if iscell(velocity)
                warning('Input will be converted to numeric array. Assure it can represent an array of length 3.')
                velocity = cell2mat(velocity);
            end
            if isnumeric(velocity) && isvector(velocity) && length(velocity)==3
                if isequal(size(velocity),[1 3])
                    velocity = velocity';
                end
                obj.initialState_.velocity = velocity;
            else
                error('Initial aircraft velocity must be a numeric vector of length 3!')
            end
        end          
        function setInitialAcceleration(obj, acceleration)
        %SETINITIALACCELERATION  Sets initial multicopter acceleration for simulation.
        %
        %   SETINITIALACCELERATION(A) Sets the initial acceleration of the
        %   multicopter in relation to inertial (absolute) frame, for simulation. 
        %   A must be a numeric vector of length 3. 
        %
        %   The initial states are used for simulation at time 0. Every
        %   time the simulation is reset, the initial states are used.
        %
        %   See also SETTIMESTEP, SETINITIALSTATE,
        %   SETINITIALPOSITION, SETINITIALATTITUDE,
        %   SETINITIALANGULARVELOCITY, SETINITIALROTORSPEEDS,
        %   SETINITIALINPUT
        
            if iscell(acceleration)
                warning('Input will be converted to numeric array. Assure it can represent an array of length 3.')
                acceleration = cell2mat(acceleration);
            end
            if isnumeric(acceleration) && isvector(acceleration) && length(acceleration)==3
                if isequal(size(acceleration),[1 3])
                    acceleration = acceleration';
                end
                obj.initialState_.acceleration = acceleration;
            else
                error('Initial aircraft acceleration must be a numeric vector of length 3!')
            end
        end       
        function setInitialAngularVelocity(obj, angularVelocity)
        %SETINITIALANGULARVELOCITY  Sets initial multicopter angular velocity for simulation.
        %
        %   SETINITIALANGULARVELOCITY(V) Sets the initial angular velocity of the
        %   multicopter in relation to body frame, for simulation. 
        %   V must be a numeric vector of length 3. 
        %
        %   If needed in relation to inertial frame, use
        %   V = matrixAtoB(initialAngularVelocity), where initialAngularVelocity is the
        %   initial angular veocity in relation to absolute reference frame and V
        %   is in relation to body frame.
        %
        %   The initial states are used for simulation at time 0. Every
        %   time the simulation is reset, the initial states are used.
        %
        %   See also MATRIXATOB, SETTIMESTEP, SETINITIALSTATE,
        %   SETINITIALPOSITION, SETINITIALATTITUDE,
        %   SETINITIALVELOCITY, SETINITIALROTORSPEEDS,
        %   SETINITIALINPUT
        
            if iscell(angularVelocity)
                warning('Input will be converted to numeric array. Assure it can represent an array of length 3.')
                angularVelocity = cell2mat(angularVelocity);
            end
            if isnumeric(angularVelocity) && isvector(angularVelocity) && length(angularVelocity)==3
                if isequal(size(angularVelocity),[1 3])
                    angularVelocity = angularVelocity';
                end
                obj.initialState_.angularVelocity = angularVelocity;
            else
                error('Initial aircraft angularVelocity must be a numeric vector of length 3!')
            end
        end
        function setInitialAngularAcceleration(obj, angularAcceleration)
        %SETINITIALANGULARVELOCITY  Sets initial multicopter angular acceleration for simulation.
        %
        %   SETINITIALANGULARVELOCITY(A) Sets the initial angular acceleration of the
        %   multicopter in relation to body frame, for simulation. 
        %   A must be a numeric vector of length 3. 
        %
        %   If needed in relation to inertial frame, use
        %   A = matrixAtoB(initialAngularAcceleration), where initialAngularAcceleration is the
        %   initial angular acceleration in relation to absolute reference frame and A
        %   is in relation to body frame.
        %
        %   The initial states are used for simulation at time 0. Every
        %   time the simulation is reset, the initial states are used.
        %
        %   See also MATRIXATOB, SETTIMESTEP, SETINITIALSTATE,
        %   SETINITIALPOSITION, SETINITIALATTITUDE,
        %   SETINITIALVELOCITY, SETINITIALROTORSPEEDS,
        %   SETINITIALINPUT
        
            if iscell(angularAcceleration)
                warning('Input will be converted to numeric array. Assure it can represent an array of length 3.')
                angularAcceleration = cell2mat(angularAcceleration);
            end
            if isnumeric(angularAcceleration) && isvector(angularAcceleration) && length(angularAcceleration)==3
                if isequal(size(angularAcceleration),[1 3])
                    angularAcceleration = angularAcceleration';
                end
                obj.initialState_.angularAcceleration = angularAcceleration;
            else
                error('Initial aircraft angularVelocity must be a numeric vector of length 3!')
            end
        end
        function setInitialRotorSpeeds(obj, rotorSpeeds)
        %SETINITIALROTORSPEEDS  Sets initial rotor speeds for simulation.
        %
        %   SETINITIALROTORSPEEDS(S) Sets the initial rotor speeds for the
        %   multicopter, for simulation. S must be a numeric vector the
        %   same length as the number of rotors already added to the
        %   multicopter.
        %
        %   The initial states are used for simulation at time 0. Every
        %   time the simulation is reset, the initial states are used.
        %
        %   See also SETTIMESTEP, SETINITIALSTATE,
        %   SETINITIALPOSITION, SETINITIALATTITUDE,
        %   SETINITIALVELOCITY, SETINITIALANGULARVELOCITY,
        %   SETINITIALINPUT
        
            if iscell(rotorSpeeds)
                warning('Input will be converted to numeric array.')
                rotorSpeeds = cell2mat(rotorSpeeds);
            end
            if isnumeric(rotorSpeeds) && isvector(rotorSpeeds) && length(rotorSpeeds)==obj.numberOfRotors_
                if isequal(size(rotorSpeeds),[1 obj.numberOfRotors_])
                    rotorSpeeds = rotorSpeeds';
                end
                aux = num2cell(rotorSpeeds);
                [obj.initialState_.rotor(1:obj.numberOfRotors_).speed] = aux{:};
            else
                error('Initial rotor speeds must be a numeric vector of length %d (number of rotors configured)!',obj.numberOfRotors_)
            end
        end      
        function setInitialRotorAccelerations(obj, rotorAccelerations)
        %SETINITIALROTORACCELERATIONS  Sets initial rotor accelerations for simulation.
        %
        %   SETINITIALROTORACCELERATIONS(S) Sets the initial rotor accelerations for the
        %   multicopter, for simulation. S must be a numeric vector the
        %   same length as the number of rotors already added to the
        %   multicopter.
        %
        %   The initial states are used for simulation at time 0. Every
        %   time the simulation is reset, the initial states are used.
        %
        %   See also SETTIMESTEP, SETINITIALSTATE,
        %   SETINITIALPOSITION, SETINITIALATTITUDE,
        %   SETINITIALVELOCITY, SETINITIALANGULARVELOCITY,
        %   SETINITIALINPUT.
        
            if iscell(rotorAccelerations)
                warning('Input will be converted to numeric array.')
                rotorAccelerations = cell2mat(rotorAccelerations);
            end
            if isnumeric(rotorAccelerations) && isvector(rotorAccelerations) && length(rotorAccelerations)==obj.numberOfRotors_
                if isequal(size(rotorAccelerations),[1 obj.numberOfRotors_])
                    rotorAccelerations = rotorAccelerations';
                end
                aux = num2cell(rotorAccelerations);
                [obj.initialState_.rotor(1:obj.numberOfRotors_).acceleration] = aux{:};
            else
                error('Initial rotor saccelerations must be a numeric vector of length %d (number of rotors configured)!',obj.numberOfRotors_)
            end
        end   
        function setInitialInput(obj, input)
        %SETINITIALINPUT  Sets initial rotor inputs for simulation.
        %
        %   SETINITIALINPUT(I) Sets the initial rotor inputs for the
        %   multicopter, for simulation. I must be a numeric vector the
        %   same length as the number of rotors already added to the
        %   multicopter.
        %
        %   Inputs may not necessarily be rotor speed set points. Check in
        %   model function.
        %
        %   The initial inputs are used for simulation at time 0. Every
        %   time the simulation is reset, the initial inputs are used.
        %
        %   See also SETTIMESTEP, SETINITIALSTATE,
        %   SETINITIALPOSITION, SETINITIALATTITUDE,
        %   SETINITIALVELOCITY, SETINITIALANGULARVELOCITY,
        %   SETINITIALROTORSPEEDS
        
            if iscell(input)
                warning('Input will be converted to numeric array.')
                input = cell2mat(input);
            end
            if isnumeric(input) && isvector(input) && length(input)==obj.numberOfRotors_
                if isequal(size(input),[1 obj.numberOfRotors_])
                    input = input';
                end
                obj.initialState_.input = input;
            else
                error('Initial input must be a numeric vector of length %d (number of rotors configured)!',obj.numberOfRotors_)
            end
        end
        function allowVerbose(obj)
            obj.verbose_ = true;
        end
        function supressVerbose(obj)
            obj.verbose_ = false;
        end
        
        %%% Get functions
        % General get
        function allProperties = multirotorProperties(obj)
        %GETALL  Returns all parameters within object.
        %
        %   GETALL() Returns a struct of all parameters inside the
        %   multicopter object. Struct is read-only and cannot change the
        %   multicopter configuration.
        %
        %   See also MASS, INERTIA, FRICTION, NUMBEROFROTORS, ROTOR
        
            allProperties.mass = obj.mass_;
            allProperties.massPError = obj.massPError_;
            allProperties.inertiaTensor = obj.inertiaTensor_;
            allProperties.inertiaTensorPError = obj.inertiaTensorPError_;
            allProperties.translationalFriction = obj.translationalFriction_;
            allProperties.translationalFrictionPError = obj.translationalFrictionPError_;
            allProperties.cgPositionError = obj.cgPositionError_;
            allProperties.numberOfRotors = obj.numberOfRotors_;
            allProperties.rotor = obj.rotor_;
            allProperties.timeStep = obj.timeStep_;
            allProperties.initialState = obj.initialState_;
            allProperties.previousState = obj.previousState_;
            allProperties.log = obj.log_;
            allProperties.isLogging = obj.isLogging_;
            allProperties.opts = obj.opts_;
            allProperties.simEffects = obj.simEffects_;
        end            
        function massValue = mass(obj)
        %MASS  Returns the multicopter mass.
        %
        %   See also GETALL, INERTIA, FRICTION, NUMBEROFROTORS, ROTOR
        
            massValue = obj.mass_;
        end
        function inertiaValue = inertia(obj)
        %INERTIA  Returns the multicopter inertia tensor.
        %   
        %   I = INERTIA() Returns the multicopter inertia tensor. I is a
        %   symmetric 3x3 matrix or an empty matrix in case inertia has not
        %   been set.
        %
        %   See also MASS, GETALL, FRICTION, NUMBEROFROTORS, ROTOR
        
            inertiaValue = obj.inertiaTensor_;
        end        
        function frictionValue = friction(obj)
        %FRICTION  Returns the multicopter translational drag matrix.
        %   
        %   A = FRICTION() Returns the multicopter translational body
        %   aerodynamic drag coefficients. A is a 3x3 matrix or an empty 
        %   matrix in case drag coefficients have not been set.
        %
        %   See also MASS, GETALL, INERTIA, NUMBEROFROTORS, ROTOR
        
            frictionValue = obj.translationalFriction_;
        end       
        function numberOfRotorsValue = numberOfRotors(obj)
        %NUMBEROFROTORS  Returns the number of rotors in the multicopter.
        %   
        %   n = NUMBEROFROTORS() Returns the number of rotors currently
        %   added to the multicopter. This value changes only when
        %   functions addRotor() or removeRotor() are called.
        %
        %   See also ADDROTOR, REMOVEROTOR, MASS, GETALL, INERTIA, FRICTION, ROTOR
        
            numberOfRotorsValue = obj.numberOfRotors_;
        end
        
        function massErrorValue = massError(obj)
        %MASSERROR  Returns the error percentage for the multicopter's mass.
        %
        %   e = MASSERROR() returns the percentage error of the multicopter's
        %   mass in relation to the nominal value set by setMass(m). e is a
        %   number usually between 0 and 1, where 0.1 represents 10%
        %   positive error and -0.1 represents 10% negative error in
        %   relation to nominal mass, for instance.
        %
        %   See also GETALL, INERTIA, FRICTION, NUMBEROFROTORS, ROTOR,
        %   SETMASSERROR.
        
            massErrorValue = obj.massPError_;
        end
        function inertiaErrorValue = inertiaError(obj)
        %INERTIAERROR  Returns the multicopter percentage error for the inertia tensor.
        %   
        %   I = INERTIAERROR() Returns a matrix containing the percentage 
        %   error for each element of the inertia tensor set using setInertia(). 
        %   I is a symmetric 3x3 matrix or an empty matrix in case the error has not
        %   been set. A value of 0.1 corresponds to 10% of error in
        %   relation to the nominal value.
        %
        %   See also MASS, GETALL, FRICTION, NUMBEROFROTORS, ROTOR,
        %   SETINERTIAERROR.
        
            inertiaErrorValue = obj.inertiaTensorPError_;
        end             
        function frictionErrorValue = frictionError(obj)
        %FRICTIONERROR  Returns the multicopter percentage error for the translational drag matrix.
        %   
        %   A = FRICTIONERROR() Returns a matrix containing the percentage
        %   error for each element of the translational body aerodynamic drag 
        %   coefficients set using friction(). A is a 3x3 matrix or an empty 
        %   matrix in case the errors have not been set. A value of 0.1 
        %   corresponds to 10% of error in relation to the nominal value.
        %
        %   See also MASS, GETALL, INERTIA, NUMBEROFROTORS, ROTOR
        
            frictionErrorValue = obj.translationalFrictionPError_;
        end    
        function cgPositionErrorValue = cgPositionError(obj)
        %CGPOSITIONERROR  Returns the error of the position of the CG 
        %
        %   error = CGPOSITIONERROR() returns the error of the position of the CG in
        %   relation to the nominal CG position in the origin of the reference
        %   frame. Error is a numeric vector of length 3.
        %
        %   See also GETALL, INERTIA, FRICTION, NUMBEROFROTORS, ROTOR
        
            cgPositionErrorValue = obj.cgPositionError_;
        end
        
        % Rotor related
        function rotorStruct = rotor(obj, rotorID)
        %ROTOR Returns the rotor struct.
        %
        %   r = ROTOR(rotorID) Returns a struct containing all rotor
        %   parameters. rotorID specifies which rotor to return the struct
        %   of. rotorID can be a single value or an array of IDs, which
        %   result in r being an array of structs.
        %
        %   ROTOR can be substituted by calls to functions that return
        %   individual rotor parameters.
        %
        %   See also ADDROTOR, REMOVEROTOR, ROTORPOSITION,
        %   ROTORORIENTATION, ROTORINERTIA, ROTORLIFTCOEFF, ROTORDRAGCOEFF,
        %   ROTORSTATUS, ROTOREFFICIENCY, ROTORMAXSPEED.
        
            rotorStruct = obj.rotor_(rotorID);
        end       
        function rotorPositionValue = rotorPosition(obj, rotorID)
        %ROTORPOSITION Returns the position of the specified rotor.
        %
        %   P = ROTORPOSITION(rotorID) Returns the position of the
        %   specified rotor in relation to body reference frame.
        %   rotorID specifies which rotor to return the position of.
        %   rotorID can be a single value or an array of IDs.
        %   P is an array of size 3xN, where N = length(rotorID) and each
        %   column vector represents the position associated with the rotor
        %   specified by the respective rotorID. 
        %
        %   See also ROTOR,ROTORORIENTATION, ROTORINERTIA, ROTORLIFTCOEFF, 
        %   ROTORDRAGCOEFF, ROTORSTATUS, ROTOREFFICIENCY, ROTORMAXSPEED.
        
            rotorPositionValue = [obj.rotor_(rotorID).position];
        end      
        function rotorOrientationValue = rotorOrientation(obj, rotorID)
        %ROTORORIENTATION Returns the orientation of the specified rotor.
        %
        %   O = ROTORORIENTATION(rotorID) Returns the orientation of the
        %   specified rotor in relation to body reference frame.
        %   rotorID specifies which rotor to return the orientation of.
        %   rotorID can be a single value or an array of IDs.
        %   O is an array of size 3xN, where N = length(rotorID) and each
        %   column vector represents the orientation associated with the rotor
        %   specified by the respective rotorID. 
        %
        %   See also ROTOR,ROTORPOSITION, ROTORINERTIA, ROTORLIFTCOEFF, 
        %   ROTORDRAGCOEFF, ROTORSTATUS, ROTOREFFICIENCY, ROTORMAXSPEED.
        
            rotorOrientationValue = [obj.rotor_(rotorID).orientation];
        end       
        function rotorInertiaValue = rotorInertia(obj, rotorID)
        %ROTORINERTIA Returns the inertia of the specified rotor.
        %
        %   I = ROTORINERTIA(rotorID) Returns the moment of inertia of the
        %   specified rotor in relation to its rotation axis.
        %   rotorID specifies which rotor to return the inertia of.
        %   rotorID can be a single value or an array of IDs.
        %   I is an array of size 1xN, where N = length(rotorID) and each
        %   vector element represents the moment of inertia associated with 
        %   the rotor specified by the respective rotorID. 
        %
        %   See also ROTOR,ROTORPOSITION, ROTORORIENTATION, ROTORLIFTCOEFF, 
        %   ROTORDRAGCOEFF, ROTORSTATUS, ROTOREFFICIENCY, ROTORMAXSPEED.
        
            rotorInertiaValue = [obj.rotor_(rotorID).inertia];
        end        
        function rotorLiftCoeffValue = rotorLiftCoeff(obj, rotorID, varargin)
        %ROTORLIFTCOEFF Returns the lift coefficient of the specified rotor.
        %
        %   L = ROTORLIFTCOEFF(rotorID) UPDATE DESCRIPTION Returns the lift coefficient of the
        %   specified rotor.
        %   rotorID specifies which rotor to return the lift coefficient of.
        %   rotorID can be a single value or an array of IDs.
        %   L is an array of size 1xN, where N = length(rotorID) and each
        %   vector element represents the lift coefficient associated with 
        %   the rotor specified by the respective rotorID. 
        %
        %   See also ROTOR,ROTORPOSITION, ROTORORIENTATION, ROTORINERTIA, 
        %   ROTORDRAGCOEFF, ROTORSTATUS, ROTOREFFICIENCY, ROTORMAXSPEED.
            rotorLiftCoeffValue = zeros(1,length(rotorID));
            if isempty(varargin)
                for it=1:length(rotorID)
                    if isempty(obj.rotor_(rotorID(it)).liftCoeff.fit)
                        rotorLiftCoeffValue(it) = obj.rotor_(rotorID(it)).liftCoeff.data;
                    else
                        rotorLiftCoeffValue(it) = max(0,obj.rotor_(rotorID(it)).liftCoeff.fit(abs(obj.previousState_.rotor(rotorID(it)).speed)));
                    end
                end
            else
                if strcmp(varargin{1},'data')
                    for it=1:length(rotorID)
                        rotorLiftCoeffValue{it} = obj.rotor_(rotorID(it)).liftCoeff.data;
                    end
                else
                    if isscalar(varargin{1})
                        for it=1:length(rotorID)
                            if isempty(obj.rotor_(rotorID(it)).liftCoeff.fit)
                                rotorLiftCoeffValue(it) = obj.rotor_(rotorID(it)).liftCoeff.data;
                            else
                                rotorLiftCoeffValue(it) = max(0,obj.rotor_(rotorID(it)).liftCoeff.fit(abs(varargin{1})));
                            end
                        end
                    else
                        if isequal(size(varargin{1}),size(rotorID))
                            for it=1:length(rotorID)
                                if isempty(obj.rotor_(rotorID(it)).liftCoeff.fit)
                                    rotorLiftCoeffValue(it) = obj.rotor_(rotorID(it)).liftCoeff.data;
                                else
                                    rotorLiftCoeffValue(it) = max(0,obj.rotor_(rotorID(it)).liftCoeff.fit(abs(varargin{1}(it))));
                                end
                            end
                        else
                            error('Wrong arguments.')
                        end
                    end
                end
            end
        end        
        function rotorDragCoeffValue = rotorDragCoeff(obj, rotorID, varargin)
        %ROTORDRAGCOEFF Returns the drag coefficient of the specified rotor.
        %
        %   D = ROTORDRAGCOEFF(rotorID) UPDATE DESCRIPTION Returns the drag coefficient of the
        %   specified rotor.
        %   rotorID specifies which rotor to return the drag coefficient of.
        %   rotorID can be a single value or an array of IDs.
        %   D is an array of size 1xN, where N = length(rotorID) and each
        %   vector element represents the drag coefficient associated with 
        %   the rotor specified by the respective rotorID. 
        %
        %   See also ROTOR,ROTORPOSITION, ROTORORIENTATION, ROTORINERTIA, 
        %   ROTORLIFTCOEFF, ROTORSTATUS, ROTOREFFICIENCY, ROTORMAXSPEED.
            rotorDragCoeffValue = zeros(1,length(rotorID));
             if isempty(varargin)
                for it=1:length(rotorID)
                    if isempty(obj.rotor_(rotorID(it)).dragCoeff.fit)
                        rotorDragCoeffValue(it) = obj.rotor_(rotorID(it)).dragCoeff.data;
                    else
                        rotorDragCoeffValue(it) = max(0,obj.rotor_(rotorID(it)).dragCoeff.fit(abs(obj.previousState_.rotor(rotorID(it)).speed)));
                    end
                end
            else
                if strcmp(varargin{1},'data')
                    for it=1:length(rotorID)
                        rotorDragCoeffValue{it} = obj.rotor_(rotorID(it)).dragCoeff.data;
                    end
                else
                    if isscalar(varargin{1})
                        for it=1:length(rotorID)
                            if isempty(obj.rotor_(rotorID(it)).dragCoeff.fit)
                                rotorDragCoeffValue(it) = obj.rotor_(rotorID(it)).dragCoeff.data;
                            else
                                rotorDragCoeffValue(it) = max(0,obj.rotor_(rotorID(it)).dragCoeff.fit(abs(varargin{1})));
                            end
                        end
                    else
                        if isequal(size(varargin{1}),size(rotorID))
                            for it=1:length(rotorID)
                                if isempty(obj.rotor_(rotorID(it)).dragCoeff.fit)
                                    rotorDragCoeffValue(it) = obj.rotor_(rotorID(it)).dragCoeff.data;
                                else
                                    rotorDragCoeffValue(it) = max(0,obj.rotor_(rotorID(it)).dragCoeff.fit(abs(varargin{1}(it))));
                                end
                            end
                        else
                            error('Wrong arguments.')
                        end
                    end
                end
            end
        end       
        function rotorMaxSpeedValue = rotorMaxSpeed(obj, rotorID)
        %ROTORMAXSPEED Returns the maximum speed of the specified rotor.
        %
        %   M = ROTORMAXSPEED(rotorID) Returns the maximum speed that the
        %   specified rotor is capable of executing.
        %   rotorID specifies which rotor to return the maximum speed of.
        %   rotorID can be a single value or an array of IDs.
        %   M is an array of size 1xN, where N = length(rotorID) and each
        %   vector element represents the maximum allowed speed associated with 
        %   the rotor specified by the respective rotorID. 
        %
        %   See also ROTOR,ROTORPOSITION, ROTORORIENTATION, ROTORINERTIA, 
        %   ROTORLIFTCOEFF, ROTORSTATUS, ROTOREFFICIENCY, ROTORDRAGCOEFF.
        
            rotorMaxSpeedValue = [obj.rotor_(rotorID).maxSpeed];
        end        
        function rotorMinSpeedValue = rotorMinSpeed(obj, rotorID)
        %ROTORMINSPEED Returns the minimum speed of the specified rotor.
        %
        %   M = ROTORMINSPEED(rotorID) Returns the minimum speed that the
        %   specified rotor is capable of executing.
        %   rotorID specifies which rotor to return the minimum speed of.
        %   rotorID can be a single value or an array of IDs.
        %   M is an array of size 1xN, where N = length(rotorID) and each
        %   vector element represents the minimum allowed speed associated with 
        %   the rotor specified by the respective rotorID. 
        %
        %   See also ROTOR,ROTORPOSITION, ROTORORIENTATION, ROTORINERTIA, 
        %   ROTORLIFTCOEFF, ROTORSTATUS, ROTOREFFICIENCY, ROTORDRAGCOEFF.
        
            rotorMinSpeedValue = [obj.rotor_(rotorID).minSpeed];
        end         
        function rotorTFValue = rotorTF(obj, rotorID)
        %ROTORTF Returns the control transfer function coefficients of the specified rotor.
        %
        %   tf = ROTORTF(rotorID) Returns the transfer function coefficients  
        %   of the specified rotor acording to G(rotorID) = a/(s^2+b*s+c),
        %   where T(:,rotorID) = [a;b;c].
        %   rotorID specifies which rotor to return the transfer function coefficients of.
        %   rotorID can be a single value or an array of IDs.
        %   tf is an array of size 3xN, where N = length(rotorID) and each
        %   column vector represents the transfer function coefficients associated with the rotor
        %   specified by the respective rotorID. 
        %
        %   See also ROTOR,ROTORORIENTATION, ROTORINERTIA, ROTORLIFTCOEFF, 
        %   ROTORDRAGCOEFF, ROTORSTATUS, ROTOREFFICIENCY, ROTORMAXSPEED.
        
            rotorTFValue = [obj.rotor_(rotorID).transferFunction];
        end     
        function rotorRm = rotorRm(obj, rotorID)
        %ROTORRM Returns the winding resistance (Ohms) of the specified rotor.
        %
        %   Rm = ROTORRM(rotorID) Returns the winding resistance of rotor
        %   rotorID.
        %   rotorID specifies which rotor to return the resistance of.
        %   rotorID can be a single value or an array of IDs.
        %   Rm is an array of size 1xN, where N = length(rotorID) and each
        %   vector element represents the winding resistance associated with 
        %   the rotor specified by the respective rotorID. 
        %
        %   The winding resistance is specified in Ohms.
        %
        %   See also ROTOR,ROTORPOSITION, ROTORORIENTATION, ROTORINERTIA, 
        %   ROTORLIFTCOEFF, ROTORSTATUS, ROTOREFFICIENCY, ROTORDRAGCOEFF.
        
            rotorRm = [obj.rotor_(rotorID).Rm];
        end  
        function rotorKt = rotorKt(obj, rotorID)
        %ROTORKT Returns the torque constant (N.m/A) of the specified rotor.
        %
        %   Kt = ROTORKT(rotorID) Returns the torque constant of rotor
        %   rotorID.
        %   rotorID specifies which rotor to return the constant of.
        %   rotorID can be a single value or an array of IDs.
        %   Kt is an array of size 1xN, where N = length(rotorID) and each
        %   vector element represents the torque constant associated with 
        %   the rotor specified by the respective rotorID. 
        %
        %   The torque constant is specified in [N.m/A].
        %
        %   See also ROTOR,ROTORPOSITION, ROTORORIENTATION, ROTORINERTIA, 
        %   ROTORLIFTCOEFF, ROTORSTATUS, ROTOREFFICIENCY, ROTORDRAGCOEFF.
        
            rotorKt = [obj.rotor_(rotorID).Kt];
        end  
        function rotorKv = rotorKv(obj, rotorID)
        %ROTORKV Returns the speed constant (rpm/V) of the specified rotor.
        %
        %   Kv = ROTORKV(rotorID) Returns the speed constant of rotor
        %   rotorID.
        %   rotorID specifies which rotor to return the constant of.
        %   rotorID can be a single value or an array of IDs.
        %   Kv is an array of size 1xN, where N = length(rotorID) and each
        %   vector element represents the speed constant associated with 
        %   the rotor specified by the respective rotorID. 
        %
        %   The speed constant is specified in [rpm/V].
        %
        %   See also ROTOR,ROTORPOSITION, ROTORORIENTATION, ROTORINERTIA, 
        %   ROTORLIFTCOEFF, ROTORSTATUS, ROTOREFFICIENCY, ROTORDRAGCOEFF.
        
            rotorKv = [obj.rotor_(rotorID).Kv];
        end 
        function rotorIo = rotorIo(obj, rotorID)
        %ROTORIO Returns the idle current rate (A/V) of the specified rotor.
        %
        %   Io = ROTORIO(rotorID) Returns the idle current rate of rotor
        %   rotorID.
        %   rotorID specifies which rotor to return the idle current rate of.
        %   rotorID can be a single value or an array of IDs.
        %   Kv is an array of size 1xN, where N = length(rotorID) and each
        %   vector element represents the idle current rate associated with 
        %   the rotor specified by the respective rotorID. 
        %
        %   The idle current rate is specified in [A/V].
        %
        %   See also ROTOR,ROTORPOSITION, ROTORORIENTATION, ROTORINERTIA, 
        %   ROTORLIFTCOEFF, ROTORSTATUS, ROTOREFFICIENCY, ROTORDRAGCOEFF.
        
            rotorIo = [obj.rotor_(rotorID).Io];
        end 
        function rotorMaxVoltage = rotorMaxVoltage(obj, rotorID)
        %ROTORMAXVOLTAGE Returns the maximum voltage allowed on specified rotor.
        %
        %   V = ROTORMAXVOLTAGE(rotorID) Returns the maximum voltage
        %   allowed on rotors rotorID.
        %   rotorID specifies which rotor to return the maximum voltages of.
        %   rotorID can be a single value or an array of IDs.
        %   V is an array of size 1xN, where N = length(rotorID) and each
        %   vector element represents the maximum voltage associated with 
        %   the rotor specified by the respective rotorID.
        %
        %   See also ROTOR,ROTORPOSITION, ROTORORIENTATION, ROTORINERTIA, 
        %   ROTORLIFTCOEFF, ROTORSTATUS, ROTOREFFICIENCY, ROTORDRAGCOEFF.
        
            rotorMaxVoltage = [obj.rotor_(rotorID).maxVoltage];
        end 
        function motorEfficiencyValue = motorEfficiency(obj, rotorID)
        %ROTOREFFICIENCY Returns the current efficiency of the specified rotor.
        %
        %   E = ROTOREFFICIENCY(rotorID) Returns the current efficiency of 
        %   the specified rotor.
        %   rotorID specifies which rotor to return the efficiency of.
        %   rotorID can be a single value or an array of IDs.
        %   E is an array of size 1xN, where N = length(rotorID) and each
        %   array element contains the efficiency associated with 
        %   the rotor specified by the respective rotorID. 
        %
        %   Rotor efficiency can be altered by calling setRotorStatus().
        %
        %   See also SETROTORSTATUS, SETROTORRUNNING, ROTOR, ROTORPOSITION, 
        %   ROTORORIENTATION, ROTORINERTIA, ROTORLIFTCOEFF, ROTORDRAGCOEFF,
        %   ROTORSTATUS, ROTORMAXSPEED.
        
            motorEfficiencyValue = [obj.rotor_(rotorID).motorEfficiency];
        end
        function propEfficiencyValue = propEfficiency(obj, rotorID)
        %PROPEFFICIENCY Returns the current propeller efficiency of the specified rotor.
        %
        %   E = PROPEFFICIENCY(rotorID) Returns the current propeller 
        %   efficiency of the specified rotor.
        %   rotorID specifies which rotor to return the prop efficiency of.
        %   rotorID can be a single value or an array of IDs.
        %   E is an array of size 1xN, where N = length(rotorID) and each
        %   array element contains the propeller efficiency associated with 
        %   the rotor specified by the respective rotorID. 
        %
        %   Propeller efficiency can be altered by calling setRotorStatus().
        %
        %   See also SETROTORSTATUS, SETROTORRUNNING, ROTOR, ROTORPOSITION, 
        %   ROTORORIENTATION, ROTORINERTIA, ROTORLIFTCOEFF, ROTORDRAGCOEFF,
        %   ROTORSTATUS, ROTORMAXSPEED.
        
            propEfficiencyValue = [obj.rotor_(rotorID).propEfficiency];
        end
              
        function rotorInertiaErrorValue = rotorInertiaError(obj, rotorID)
        %ROTORINERTIAERROR Returns the percentage error for the rotor inertia.
        %
        %   I = ROTORINERTIAERROR(rotorID) Returns the percentage of error 
        %   of the moment of inertia of the specified rotor in relation to 
        %   the nominal value set using setRotorInertia().
        %   rotorID specifies which rotor to return the error of.
        %   rotorID can be a single value or an array of IDs.
        %   I is an array of size 1xN, where N = length(rotorID) and each
        %   vector element represents the percentage error of the moment of 
        %   inertia associated with the rotor specified by the respective rotorID. 
        %   0.1 represents a 10% error in relation to the nominal
        %   value.
        %
        %   See also ROTOR,ROTORPOSITION, ROTORORIENTATION, ROTORLIFTCOEFF, 
        %   ROTORDRAGCOEFF, ROTORSTATUS, ROTOREFFICIENCY, ROTORMAXSPEED.
        
            rotorInertiaErrorValue = [obj.rotor_(rotorID).inertiaPError];
        end          
        function rotorLiftCoeffErrorValue = rotorLiftCoeffError(obj, rotorID)
        %ROTORLIFTCOEFFERROR Returns the percentage error for the rotor lift coefficient.
        %
        %   L = ROTORLIFTCOEFF(rotorID) Returns the percentage of error 
        %   of the lift coefficient of the specified rotor in relation to 
        %   the nominal value set using setRotorLiftCoeff().
        %   rotorID specifies which rotor to return the error of.
        %   rotorID can be a single value or an array of IDs.
        %   L is an array of size 1xN, where N = length(rotorID) and each
        %   vector element represents the percentage error of the lift 
        %   coefficient associated with the rotor specified by the respective rotorID. 
        %   0.1 represents a 10% error in relation to the nominal
        %   value. 
        %
        %   See also ROTOR,ROTORPOSITION, ROTORORIENTATION, ROTORINERTIA, 
        %   ROTORDRAGCOEFF, ROTORSTATUS, ROTOREFFICIENCY, ROTORMAXSPEED.
        
            rotorLiftCoeffErrorValue = [obj.rotor_(rotorID).liftCoeffPError];
        end        
        function rotorDragCoeffErrorValue = rotorDragCoeffError(obj, rotorID)
        %ROTORDRAGCOEFFERROR Returns the percentage error for the rotor drag coefficient.
        %
        %   D = ROTORDRAGCOEFF(rotorID) Returns the percentage of error 
        %   of the drag coefficient of the specified rotor in relation to 
        %   the nominal value set using setRotorDragCoeff().
        %   rotorID specifies which rotor to return the error of.
        %   rotorID can be a single value or an array of IDs.
        %   D is an array of size 1xN, where N = length(rotorID) and each
        %   vector element represents the percentage error of the drag
        %   coefficient associated with the rotor specified by the respective rotorID. 
        %   0.1 represents a 10% error in relation to the nominal
        %   value. 
        %
        %   See also ROTOR,ROTORPOSITION, ROTORORIENTATION, ROTORINERTIA, 
        %   ROTORLIFTCOEFF, ROTORSTATUS, ROTOREFFICIENCY, ROTORMAXSPEED.
        
            rotorDragCoeffErrorValue = [obj.rotor_(rotorID).dragCoeffPError];
        end               
        function rotorTFErrorValue = rotorTFError(obj, rotorID)
        %ROTORTFERROR Returns the percentage error for the rotor transfer function coefficients.
        %
        %   tf = ROTORTFERROR(rotorID) Returns the percentage error for the
        %   transfer function coefficients of the specified rotor.
        %   rotorID specifies which rotor to return the errors of.
        %   rotorID can be a single value or an array of IDs.
        %   tf is an array of size 3xN, where N = length(rotorID) and each
        %   column vector represents the percentage errors for the transfer 
        %   function coefficients associated with the rotor
        %   specified by the respective rotorID. 0.1 represents a 10% error 
        %   in relation to the nominal value. 
        %
        %   See also ROTOR,ROTORORIENTATION, ROTORINERTIA, ROTORLIFTCOEFF, 
        %   ROTORDRAGCOEFF, ROTORSTATUS, ROTOREFFICIENCY, ROTORMAXSPEED.
        
            rotorTFErrorValue = [obj.rotor_(rotorID).transferFunctionPError];
        end  
        
        % Simulation related
        function timeStepValue = timeStep(obj)
        %TIMESTEP  Returns current simulation time step in seconds.
        %
        %   See also INITIALSTATE, PREVIOUSSTATE, LOG, ISLOGGING, ISRUNNING
        
            timeStepValue = obj.timeStep_;
        end 
        function initialStateStruct = initialState(obj)
        %INITIALSTATE Returns the initial state struct.
        %
        %   i = INITIALSTATE() Returns a struct containing all
        %   multicopter's initial states, rotor speeds and inputs.
        %
        %   INITIALSTATE can be substituted by calls to functions that return
        %   individual values such as position, attitude, velocity, etc.
        %
        %   See also INITIALPOSITION, INITIALATTITUDE, INITIALVELOCITY,
        %   INITIALANGULARVELOCITY, INITIALROTORSPEED, INITIALINPUT,
        %   TIMESTEP, PREVIOUSSTATE, LOG, ISLOGGING, ISRUNNING
        
            initialStateStruct = obj.initialState_;
        end       
        function initialStateVectorValue = initialStateVector(obj)
        %INITIALSTATEVECTOR Returns the initial state in vector form.
        %
        %   i = INITIALSTATEVECTOR() Returns a vector containing all
        %   multicopter's initial states, rotor speeds and accelerations. i is 
        %   a numeric vector of length 13+2*nR, where nR is the number of 
        %   rotors already added to this multicopter.
        %   
        %   i is comprised of position, attitude, velocity, angular
        %   velocity, rotor speeds and inputs, in this order.
        %
        %   INITIALSTATEVECTOR can be substituted by calls to functions that return
        %   individual values such as position, attitude, velocity, etc.
        %
        %   See also INITIALPOSITION, INITIALATTITUDE, INITIALVELOCITY,
        %   INITIALANGULARVELOCITY, INITIALROTORSPEED, INITIALINPUT,
        %   TIMESTEP, PREVIOUSSTATE, LOG, ISLOGGING, ISRUNNING
        
            initialStateVectorValue =   [obj.initialState_.position;
                                        obj.initialState_.attitude;
                                        obj.initialState_.velocity;
                                        obj.initialState_.angularVelocity];
            if ~isempty(obj.initialState_.rotor)
                initialStateVectorValue = [initialStateVectorValue; [obj.initialState_.rotor(:).speed]'];
                initialStateVectorValue = [initialStateVectorValue; [obj.initialState_.rotor(:).acceleration]'];
            end
        end        
        function initialPositionValue = initialPosition(obj)
        %INITIALPOSITION Returns the initial position of the multicopter.
        %
        %   P = INITIALPOSITION() Returns the initial position of the
        %   multicopter in relation to the inertial (absolute) reference
        %   frame. P is a column vector of length 3.
        %
        %   See also INITIALATTITUDE, INITIALVELOCITY,
        %   INITIALANGULARVELOCITY, INITIALROTORSPEED, INITIALINPUT,
        %   TIMESTEP, PREVIOUSSTATE, LOG, ISLOGGING, ISRUNNING
        
            initialPositionValue = obj.initialState_.position;
        end        
        function initialAttitudeValue = initialAttitude(obj)
        %INITIALATTITUDE Returns the initial attitude of the multicopter.
        %
        %   A = INITIALATTITUDE() Returns the initial attitude of the
        %   multicopter in quaternion form, in relation to the inertial 
        %   (absolute) reference frame. A is a column vector of length 4.
        %
        %   In case euler angles are needed, use A =
        %   toEuler(initialAttitude()), where A is an euler angles vector 
        %   of length 3.
        %
        %   See also TOEULER, INITIALPOSITION, INITIALVELOCITY,
        %   INITIALANGULARVELOCITY, INITIALROTORSPEED, INITIALINPUT,
        %   TIMESTEP, PREVIOUSSTATE, LOG, ISLOGGING, ISRUNNING
        
            initialAttitudeValue = obj.initialState_.attitude;
        end        
        function initialVelocityValue = initialVelocity(obj)
        %INITIALVELOCITY Returns the initial velocity of the multicopter.
        %
        %   V = INITIALVELOCITY() Returns the initial velocity of the
        %   multicopter in relation to the inertial (absolute) reference
        %   frame. V is a column vector of length 3.
        %
        %   See also INITIALPOSITION, INITIALATTITUDE,
        %   INITIALANGULARVELOCITY, INITIALROTORSPEED, INITIALINPUT,
        %   TIMESTEP, PREVIOUSSTATE, LOG, ISLOGGING, ISRUNNING
        
            initialVelocityValue = obj.initialState_.velocity;
        end       
        function initialAccelerationValue = initialAcceleration(obj)
        %INITIALACCELERATION Returns the initial acceleration of the multicopter.
        %
        %   A = INITIALACCELERATION() Returns the initial acceleration of the
        %   multicopter in relation to the inertial (absolute) reference
        %   frame. A is a column vector of length 3.
        %
        %   See also INITIALPOSITION, INITIALATTITUDE,
        %   INITIALANGULARVELOCITY, INITIALROTORSPEED, INITIALINPUT,
        %   TIMESTEP, PREVIOUSSTATE, LOG, ISLOGGING, ISRUNNING
        
            initialAccelerationValue = obj.initialState_.acceleration;
        end     
        function initialAngularVelocityValue = initialAngularVelocity(obj)
        %INITIALANGULARVELOCITY Returns the initial angular velocity of the multicopter.
        %
        %   V = INITIALANGULRVELOCITY() Returns the initial angular velocity 
        %   of the multicopter in relation to the body reference
        %   frame. V is a column vector of length 3.
        %
        %   If needed, use V = matrixBtoA(INITIALANGULARVELOCITY()), where
        %   V is the angular velocity in relation to the absolute reference
        %   frame.
        %
        %   See also MATRIXBTOA, INITIALPOSITION, INITIALATTITUDE,
        %   INITIALVELOCITY, INITIALROTORSPEED, INITIALINPUT,
        %   TIMESTEP, PREVIOUSSTATE, LOG, ISLOGGING, ISRUNNING
        
            initialAngularVelocityValue = obj.initialState_.angularVelocity;
        end 
        function initialAngularAccelerationValue = initialAngularAcceleration(obj)
        %INITIALANGULARACCELERATION Returns the initial angular acceleration of the multicopter.
        %
        %   A = INITIALANGULRACCELERATION() Returns the initial angular acceleration
        %   of the multicopter in relation to the body reference
        %   frame. A is a column vector of length 3.
        %
        %   If needed, use A= matrixBtoA(INITIALANGULARACCELERATION()), where
        %   A is the angular acceleration in relation to the absolute reference
        %   frame.
        %
        %   See also MATRIXBTOA, INITIALPOSITION, INITIALATTITUDE,
        %   INITIALVELOCITY, INITIALROTORSPEED, INITIALINPUT,
        %   TIMESTEP, PREVIOUSSTATE, LOG, ISLOGGING, ISRUNNING
        
            initialAngularAccelerationValue = obj.initialState_.angularAcceleration;
        end 
        function initialRotorSpeedVector = initialRotorSpeed(obj, rotorID)
        %INITIALROTORSPEED Returns the initial rotor speeds.
        %
        %   V = INITIALROTORSPEED(rotorID) Returns the initial rotor speeds 
        %   of the specified rotors.   
        %   rotorID specifies which rotor to return the speed of.
        %   rotorID can be a single value or an array of IDs.  V is a
        %   vector the length of the number of rotors in the multicopter.
        %
        %   See also INITIALPOSITION, INITIALATTITUDE,
        %   INITIALVELOCITY, INITIALANGULARVELOCITY, INITIALINPUT,
        %   TIMESTEP, PREVIOUSSTATE, LOG, ISLOGGING, ISRUNNING
        
            if ~isempty(obj.initialState_.rotor)
                initialRotorSpeedVector = [obj.initialState_.rotor(rotorID).speed];
            else
                initialRotorSpeedVector = [];
            end
        end   
        function initialRotorAccelerationVector = initialRotorAcceleration(obj, rotorID)
        %INITIALROTORACCELERATION Returns the initial rotor accelerations.
        %
        %   V = INITIALROTORACCELERATION(rotorID) Returns the initial rotor
        %   accelerations of the specified rotors.   
        %   rotorID specifies which rotor to return the acceleration of.
        %   rotorID can be a single value or an array of IDs.  V is a
        %   vector the length of the number of rotors in the multicopter.
        %
        %   See also INITIALPOSITION, INITIALATTITUDE,
        %   INITIALVELOCITY, INITIALANGULARVELOCITY, INITIALINPUT,
        %   TIMESTEP, PREVIOUSSTATE, LOG, ISLOGGING, ISRUNNING
        
            if ~isempty(obj.initialState_.rotor)
                initialRotorAccelerationVector = [obj.initialState_.rotor(rotorID).acceleration];
            else
                initialRotorAccelerationVector = [];
            end
        end  
        function initialInputVector = initialInput(obj)
        %INITIALINPUT Returns the initial inputs.
        %
        %   I = INITIALINPUT() Returns the initial inputs of the multicopter.
        %   V is a vector the length of the number of rotors in the multicopter.
        %
        %   See also INITIALPOSITION, INITIALATTITUDE,
        %   INITIALVELOCITY, INITIALANGULARVELOCITY, INITIALROTORSPEEDS,
        %   TIMESTEP, PREVIOUSSTATE, LOG, ISLOGGING, ISRUNNING
        
            initialInputVector = obj.initialState_.input;
        end
        
        function previousStateStruct = previousState(obj)
        %PREVIOUSSTATE Returns the previous state struct.
        %
        %   i = PREVIOUSSTATE() Returns a struct containing all
        %   multicopter's previous states, rotor speeds and inputs.
        %
        %   PREVIOUSSTATE can be substituted by calls to functions that return
        %   individual values such as position, attitude, velocity, etc.
        %
        %   See also PREVIOUSPOSITION, PREVIOUSATTITUDE, PREVIOUSVELOCITY,
        %   PREVIOUSANGULARVELOCITY, PREVIOUSACCELERATION,
        %   PREVIOUSANGULARACCELERATION,
        %   PREVIOUSROTORSPEED, PREVIOUSINPUT,
        %   PREVIOUSTIME, TIMESTEP, INITIALSTATE, LOG, ISLOGGING, ISRUNNING
        
            previousStateStruct = obj.previousState_;
        end       
        function previousStateVectorValue = previousStateVector(obj)
        %PREVIOUSSTATEVECTOR Returns the previous state in vector form.
        %
        %   i = PREVIOUSSTATEVECTOR() Returns a vector containing all
        %   multicopter's previous states, rotor speeds, accelerations and inputs. i is 
        %   a numeric vector of length 20+3*nR, where nR is the number of 
        %   rotors already added to this multicopter.
        %   
        %   i is comprised of position, attitude, velocity, angular
        %   velocity, acceleration, angular acceleration, rotor speeds,
        %   rotor accelerations, inputs and previous time, in this order.
        %
        %   PREVIOUSSTATEVECTOR can be substituted by calls to functions that return
        %   individual values such as position, attitude, velocity, etc.
        %
        %   Previous states, or the term previous, refer to the last values
        %   used in the last simulation run. The time of these states can
        %   be accessed using previousTime() or the last element of the vector
        %   of states.
        %
        %   See also PREVIOUSPOSITION, PREVIOUSATTITUDE, PREVIOUSVELOCITY,
        %   PREVIOUSANGULARVELOCITY, PREVIOUSACCELERATION,
        %   PREVIOUSANGULARACCELERATION, PREVIOUSROTORSPEED, PREVIOUSINPUT,
        %   PREVIOUSTIME, TIMESTEP, PREVIOUSSTATE, LOG, ISLOGGING, ISRUNNING
        
            previousStateVectorValue =   [obj.previousState_.position;
                                        obj.previousState_.attitude;
                                        obj.previousState_.velocity;
                                        obj.previousState_.angularVelocity;
                                        obj.previousState_.acceleration;
                                        obj.previousState_.angularAcceleration];
            if ~isempty(obj.previousState_.rotor)
                previousStateVectorValue = [previousStateVectorValue; obj.previousState_.rotor(:).speed'];
                previousStateVectorValue = [previousStateVectorValue; obj.previousState_.rotor(:).acceleration'];
                previousStateVectorValue = [previousStateVectorValue; obj.previousState_.input'];
            end
            previousStateVectorValue = [previousStateVectorValue; obj.previousState_.time];
        end               
        function previousPositionValue = previousPosition(obj)
        %PREVIOUSPOSITION Returns the previous position of the multicopter.
        %
        %   P = PREVIOUSPOSITION() Returns the previou position of the
        %   multicopter in relation to the inertial (absolute) reference
        %   frame. P is a column vector of length 3.
        %
        %   Previous states, or the term previous, refer to the last values
        %   used in the last simulation run. The time of these states can
        %   be accessed using previousTime().
        %
        %   See also PREVIOUSSTATE, PREVIOUSATTITUDE, PREVIOUSVELOCITY,
        %   PREVIOUSANGULARVELOCITY, PREVIOUSROTORSPEED, PREVIOUSINPUT,
        %   PREVIOUSTIME, TIMESTEP, INITIALSTATE, LOG, ISLOGGING, ISRUNNING
        
            previousPositionValue = obj.previousState_.position;
        end        
        function previousAttitudeValue = previousAttitude(obj)
        %PREVIOUSATTITUDE Returns the previous attitude of the multicopter.
        %
        %   A = PREVIOUSATTITUDE() Returns the previous attitude of the
        %   multicopter in quaternion form, in relation to the inertial 
        %   (absolute) reference frame. A is a column vector of length 4.
        %
        %   In case euler angles are needed, use A =
        %   toEuler(initialAttitude()), where A is an euler angles vector 
        %   of length 3.
        %
        %   Previous states, or the term previous, refer to the last values
        %   used in the last simulation run. The time of these states can
        %   be accessed using previousTime().
        %
        %   See also PREVIOUSSTATE, PREVIOUSPOSITION, PREVIOUSVELOCITY,
        %   PREVIOUSANGULARVELOCITY, PREVIOUSROTORSPEED, PREVIOUSINPUT,
        %   PREVIOUSTIME, TIMESTEP, INITIALSTATE, LOG, ISLOGGING, ISRUNNING
        
            previousAttitudeValue = obj.previousState_.attitude;
        end        
        function previousVelocityValue = previousVelocity(obj)
        %PREVIOUSVELOCITY Returns the previous velocity of the multicopter.
        %
        %   V = PREVIOUSVELOCITY() Returns the previous velocity of the
        %   multicopter in relation to the inertial (absolute) reference
        %   frame. V is a column vector of length 3.
        %
        %   Previous states, or the term previous, refer to the last values
        %   used in the last simulation run. The time of these states can
        %   be accessed using previousTime().
        %
        %   See also PREVIOUSSTATE, PREVIOUSPOSITION, PREVIOUSATTITUDE,
        %   PREVIOUSANGULARVELOCITY, PREVIOUSROTORSPEED, PREVIOUSINPUT,
        %   PREVIOUSTIME, TIMESTEP, INITIALSTATE, LOG, ISLOGGING, ISRUNNING
        
            previousVelocityValue = obj.previousState_.velocity;
        end        
        function previousAngularVelocityValue = previousAngularVelocity(obj)
        %PREVIOUSANGULARVELOCITY Returns the previous angular velocity of the multicopter.
        %
        %   V = PREVIOUSANGULARVELOCITY() Returns the previous angular velocity 
        %   of the multicopter in relation to the body reference
        %   frame. V is a column vector of length 3.
        %
        %   If needed, use V = matrixBtoA(PREVIOUSANGULARVELOCITY()), where
        %   V is the angular velocity in relation to the absolute reference
        %   frame.
        %
        %   Previous states, or the term previous, refer to the last values
        %   used in the last simulation run. The time of these states can
        %   be accessed using previousTime().
        %
        %   See also PREVIOUSSTATE, PREVIOUSPOSITION, PREVIOUSATTITUDE,
        %   PREVIOUSVELOCITY, PREVIOUSROTORSPEED, PREVIOUSINPUT,
        %   PREVIOUSTIME, TIMESTEP, INITIALSTATE, LOG, ISLOGGING, ISRUNNING
        
            previousAngularVelocityValue = obj.previousState_.angularVelocity;
        end        
        function previousAccelerationValue = previousAcceleration(obj)
        %PREVIOUSACCELERATION Returns the previous acceleration of the multicopter.
        %
        %   A = PREVIOUSACCELERATION() Returns the previous acceleration of the
        %   multicopter in relation to the inertial (absolute) reference
        %   frame. A is a column vector of length 3.
        %
        %   Previous states, or the term previous, refer to the last values
        %   used in the last simulation run. The time of these states can
        %   be accessed using previousTime().
        %
        %   See also PREVIOUSSTATE, PREVIOUSPOSITION, PREVIOUSATTITUDE,
        %   PREVIOUSANGULARVELOCITY, PREVIOUSANGULARACCELERATION,
        %   PREVIOUSROTORSPEED, PREVIOUSINPUT,
        %   PREVIOUSTIME, TIMESTEP, INITIALSTATE, LOG, ISLOGGING, ISRUNNING
        
            previousAccelerationValue = obj.previousState_.acceleration;
        end        
        function previousAngularAccelerationValue = previousAngularAcceleration(obj)
        %PREVIOUSANGULARACCELERATION Returns the previous angular velocity of the multicopter.
        %
        %   A = PREVIOUSANGULARACCELERATION() Returns the previous angular
        %   acceleration of the multicopter in relation to the body reference
        %   frame. A is a column vector of length 3.
        %
        %   If needed, use A = matrixBtoA(PREVIOUSANGULARACCELERATION()), where
        %   A is the angular acceleration in relation to the absolute reference
        %   frame.
        %
        %   Previous states, or the term previous, refer to the last values
        %   used in the last simulation run. The time of these states can
        %   be accessed using previousTime().
        %
        %   See also PREVIOUSSTATE, PREVIOUSPOSITION, PREVIOUSATTITUDE,
        %   PREVIOUSVELOCITY, PREVIOUSACCELERATION, PREVIOUSROTORSPEED, PREVIOUSINPUT,
        %   PREVIOUSTIME, TIMESTEP, INITIALSTATE, LOG, ISLOGGING, ISRUNNING
        
            previousAngularAccelerationValue = obj.previousState_.angularAcceleration;
        end    
        function previousRotorSpeedVector = previousRotorSpeed(obj, rotorID)
        %PREVIOUSROTORSPEED Returns the previous rotor speeds.
        %
        %   V = PREVIOUSROTORSPEED(rotorID) Returns the previous rotor speeds
        %   of the specified rotors.        
        %   rotorID specifies which rotor to return the speed of.
        %   rotorID can be a single value or an array of IDs.   V is a
        %   vector the length of the number of rotors in the multicopter.
        %
        %   Previous states, or the term previous, refer to the last values
        %   used in the last simulation run. The time of these states can
        %   be accessed using previousTime().
        %
        %   See also PREVIOUSSTATE, PREVIOUSPOSITION, PREVIOUSATTITUDE,
        %   PREVIOUSVELOCITY, PREVIOUSANGULARVELOCITY, PREVIOUSINPUT,
        %   PREVIOUSTIME, TIMESTEP, INITIALSTATE, LOG, ISLOGGING, ISRUNNING
        
            if ~isempty(obj.previousState_.rotor)
                previousRotorSpeedVector = [obj.previousState_.rotor(rotorID).speed];
            else
                previousRotorSpeedVector = [];
            end
        end       
        function previousRotorAccelerationVector = previousRotorAcceleration(obj, rotorID)
        %PREVIOUSROTORACCELERATION Returns the previous rotor accelerations.
        %
        %   V = PREVIOUSROTORACCELERATION(rotorID) Returns the previous rotor 
        %   accelerations of the specified rotors.        
        %   rotorID specifies which rotor to return the acceleration of.
        %   rotorID can be a single value or an array of IDs.   V is a
        %   vector the length of the number of rotors in the multicopter.
        %
        %   Previous states, or the term previous, refer to the last values
        %   used in the last simulation run. The time of these states can
        %   be accessed using previousTime().
        %
        %   See also PREVIOUSSTATE, PREVIOUSPOSITION, PREVIOUSATTITUDE,
        %   PREVIOUSVELOCITY, PREVIOUSANGULARVELOCITY, PREVIOUSINPUT,
        %   PREVIOUSTIME, TIMESTEP, INITIALSTATE, LOG, ISLOGGING, ISRUNNING
        
            if ~isempty(obj.previousState_.rotor)
                previousRotorAccelerationVector = [obj.previousState_.rotor(rotorID).acceleration];
            else
                previousRotorAccelerationVector = [];
            end
        end   
        function previousInputVector = previousInput(obj)
        %PREVIOUSINPUT Returns the previous inputs.
        %
        %   V = PREVIOUSINPUT() Returns the previous inputs of the multicopter.
        %   V is a vector the length of the number of rotors in the multicopter.
        %
        %   Previous states, or the term previous, refer to the last values
        %   used in the last simulation run. The time of these states can
        %   be accessed using previousTime().
        %
        %   See also PREVIOUSSTATE, PREVIOUSPOSITION, PREVIOUSATTITUDE,
        %   PREVIOUSVELOCITY, PREVIOUSANGULARVELOCITY, PREVIOUSROTORSPEED,
        %   PREVIOUSTIME, TIMESTEP, INITIALSTATE, LOG, ISLOGGING, ISRUNNING
        
            previousInputVector = obj.previousState_.input;
        end        
        function previousTimeValue = previousTime(obj)
        %PREVIOUSTIME Returns the time of the last simulated values.
        %
        %   t = PREVIOUSTIME() Returns the time of the last simulation
        %   step. t is in seconds and is not necessarily in real-time.
        %
        %   Previous states, or the term previous, refer to the last values
        %   used in the last simulation run. 
        %
        %   See also PREVIOUSSTATE, PREVIOUSPOSITION, PREVIOUSATTITUDE,
        %   PREVIOUSVELOCITY, PREVIOUSANGULARVELOCITY, PREVIOUSROTORSPEED,
        %   PREVIOUSINPUT, TIMESTEP, INITIALSTATE, LOG, ISLOGGING, ISRUNNING
        
            previousTimeValue = obj.previousState_.time;
        end
        
        function logStruct = log(obj)
        %LOG Returns the history of all simulation steps.
        %
        %   H = LOG() Returns all previous simulation results and inputs
        %   for when logging was habilitated. 
        %   H is a matrix where each row represent a simulation step and
        %   each column a time, state or input value.
        %   H is comprised of [time, position, attitude, velocity, angular
        %   velocity, rotor speeds, rotor accelerations, rotor speeds set
        %   points, inputs], in this order. atitude is in quaternion form.
        %
        %   Logging can be controlled by calling startLogging() and stopLogging()
        %   functions.
        %
        %   In case a rotor has been added or removed during simulation,
        %   log matrix will be formatted to fit all rotor inputs, which
        %   means addind zeros to places that should have values from
        %   removed rotors.
        %
        %   See also STARTLOGGING, STOPLOGGING, ISLOGGING, CLEARLOG,
        %   REMOVEROTOR, ADDROTOR, RUN
        
            logStruct = obj.log_;
        end        
        function isLoggingFlag = isLogging(obj)
        %ISLOGGING Indicates whether simulation is being logged or not.
        %
        %   Returns true for when logging is activated and false otherwise.
        %
        %   See also LOG, ISRUNNING, CLEARLOG, RUN
        
            isLoggingFlag = obj.isLogging_;
        end        
        function isRunningFlag = isRunning(obj)            
        %ISRUNNING Indicates whether simulation has started or not.
        %
        %   Returns true in case simulation has already started and
        %   previous states are different from initial states. Returns
        %   false in case no simulation has been started or in case it has
        %   been reset.
        %
        %   See also LOG, ISLOGGING, CLEARLOG, RUN
        
            isRunningFlag = obj.isRunning_;
        end
        
        %%% Remove/Delete Functions    
        function clearModel(obj)          
        %CLEARMODEL Clears all internal properties and makes model not
        %appropriate for simulation.
        %
        %   See also REMOVEROTOR, CLEARLOG, SETMASS, SETINERTIA,
        %   SETFRICTION, SETINITIALSTATE, ADDROTOR, SETTIMESTEP
        
            obj.mass_ = [];
            obj.massPError_ = [];
            obj.inertiaTensor_ = [];
            obj.inertiaTensorPError_ = [];
            obj.translationalFriction_ = [];
            obj.translationalFrictionPError_ = [];
            obj.cgPositionError_ = [];
            obj.numberOfRotors_ = 0;
            obj.rotor_ = [];
            obj.timeStep_ = [];
            obj.initialState_ = [];
            obj.previousState_ = [];
            obj.log_ = [];
            obj.isLogging_ = [];
        end        
        function removeRotor(obj, varargin)        
        %REMOVEROTOR Removes rotor and related variables from multicopter.
        %
        %   REMOVEROTOR() removes the last added motor and all related
        %   variables, except log values.
        %
        %   REMOVEROTOR(rotorID) removes the rotors specified by rotorID.
        %   rotorID may be a scalar or vector with the IDs of the rotors to
        %   be remove.
        %
        %   Caution: When a rotor is removed all other rotors change IDs.
        %   Example: considers a model with rotors [1 2 3 4]. Removing
        %   rotor 3 will make rotor 4 to become rotor 3, such that now the
        %   model is comprised of rotors [1 2 3].
        %   
        %   REMOVEROTOR can be used during simulation, but in most cases it is
        %   suggested to add and set all rotors during initial configuration 
        %   and set them as fail or running acordingly. 
        %
        %   Removing a rotor during simulation will make the log matrix to
        %   change size and waste memory, but keeping track of
        %   modifications.
        %
        %   See also ADDROTOR, CLEARLOG, SETROTOR
        
            if isempty(varargin)
                rotorID = obj.numberOfRotors_;
            else
                rotorID = varargin{:};
            end
            obj.rotor_(rotorID) = [];
            obj.numberOfRotors_ = obj.numberOfRotors_ - length(rotorID);
            obj.initialState_.rotor(rotorID)=[];
            if rotorID<=length(obj.initialState_.input)
                obj.initialState_.input(rotorID) = [];
            end
            if ~isempty(obj.previousState_.rotor) && rotorID<=length(obj.previousState_.rotor)
                obj.previousState_.rotor(rotorID)=[];
            end
            if ~isempty(obj.previousState_.input) && rotorID<=length(obj.previousState_.input)
                obj.previousState_.input(rotorID)=[];
            end
        end
        
        %%% Simulation functions
        function startLogging(obj)
        %STARTLOGGING Enables logging states and inputs during simulation.
        %
        % See also LOG, STOPLOGGING, CLEARLOG, RUN
        
            obj.isLogging_ = 1;
        end       
        function stopLogging(obj)
        %STOPLOGGING Disables logging states and inputs during simulation.
        %
        % See also LOG, STARTLOGGING, CLEARLOG, RUN
        
            obj.isLogging_ = 0;
        end       
        function clearLog(obj)
        %CLEARLOG Clears all log history
        %
        % See also LOG, STOPLOGGING, STARTLOGGING, RUN        
            obj.log_ = [];
            obj.log_.time = [];
            obj.log_.position = [];
            obj.log_.attitude = [];
            obj.log_.velocity = [];
            obj.log_.angularVelocity = [];
            obj.log_.input = [];
            for i=1:obj.numberOfRotors_
                obj.log_.rotor(i).setPoint = [];
                obj.log_.rotor(i).speed = [];
                obj.log_.rotor(i).acceleration = [];
                obj.log_.rotor(i).torque = [];
            end
            obj.log_.power = [];
        end
        function [t,output] = run(obj, input, varargin)
        %RUN Simulates multicopter
        %
        %   [t,output] = RUN(I) Simulates the multicopter for inputs I. The
        %   number of rows of I must be the same as the number of rotors
        %   configured in the multicopter, even if they are set as fail. I
        %   may be a column vector or an array comprised of column vector
        %   inputs. Each column vector is simulated for the increment of
        %   one simulation time step.
        %
        %   Example: I = [ 1, 2, 3, 4, 5;
        %                 -1,-2,-3,-4,-5;
        %                  1, 2, 3, 4, 5;
        %                 -1,-2,-3,-4,-5];
        %
        %   is a 5 step input for a quadricopter (values not important). 
        %   The simulation times and their corresponding simulation inputs are:
        %   time = [0, timeStep, 2*timeStep, 3*timeStep, 4*timeStep, 5*timeStep]
        %   inputs = [initialInputs, I(:,1),I(:,2),I(:,3),I(:,4),I(:,5)]
        %
        %   [t,output] = RUN(I,T) Simulates the multicopter or continues 
        %   simulation for inputs I and times T. 
        %   I is specified just as in RUN(I) and T may be scalar or a 
        %   numeric vector of any length. 
        %   Each element of T specify the simulation time for when the 
        %   corresponding column input in I must be "applied" to the 
        %   multicopter. If length(T)>size(I,2), the simulation will run 
        %   until the last value of T. If length(T)<=size(I,2), the
        %   simulation will run until the last input, considering the
        %   inpput times as simulation timesteps.
        %   Elements of T must be in ascending order (it's a time vector,
        %   after all), but may be arbitrary. 
        %   Input values are held until the time for the next input to be
        %   "applied". Input changes are made in the closest simulation
        %   step time to the time specified in T for that input, but not
        %   necessarily at the exact time in T.
        %
        %   Example: For I = [ 1, 2, 3, 4, 5;   T = [0.9, 2, 3]; and timeStep = 0.5;
        %                     -1,-2,-3,-4,-5;
        %                      1, 2, 3, 4, 5;
        %                     -1,-2,-3,-4,-5];
        %    
        %   The simulation times and their corresponding simulation inputs are:
        %   time = [0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0] (in case it is the first run)
        %   inputs = [initialInputs, initialInputs, I(:,1),I(:,1),I(:,2),I(:,2),I(:,3),I(:,4),I(:,5)]
        %
        %   Example: For I = [ 1, 2, 3, 4, 5;   T = [0.9, 2, 3, 4, 5, 6.6, 7.2]; and timeStep = 0.5;
        %                     -1,-2,-3,-4,-5;
        %                      1, 2, 3, 4, 5;
        %                     -1,-2,-3,-4,-5];
        %    
        %   The simulation times and their corresponding simulation inputs are:
        %   time = [0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0] (in case it is the first run)
        %   inputs = [initialInputs, initialInputs, I(:,1),I(:,1),I(:,2),I(:,2),I(:,3),I(:,3),I(:,4),I(:,4),I(:,5),I(:,5),I(:,5),I(:,5),I(:,5)]
        %
        %   If run is called more than once, the next call continues the
        %   simulation considering the previous time, states and inputs
        %   from last simulation run.
        %
        %   For online multicopter control simulation, suggested use is
        %   within a control loop such as (simplified):
        %   while(t<=simulationTime)
        %       %"Sensing" section
        %
        %       %Control calculations
        %
        %       %Motor applications and plant simulation
        %       [t,output] = multicopter.run(nextInput, nextControlTimeStep);
        %   end
        %
        %   Where simulationTime is the maximum time to be simulated,
        %   nextInput is a column vector for next control application
        %   and nextControlTimeStep is t+controlTimeStep. controlTimeStep
        %   must be greater than simulation's timeStep and represents the
        %   loop time of an embedded microcontroler or ESC in the
        %   multicopter.
        %
        %   t is a vector of time values for each simulation time step starting 
        %   from previousTime+timeStep. 
        %   output are the simulation results, returned as an array. Each 
        %   row in output corresponds to the simulation result at the time 
        %   returned in the corresponding row of t.
        %   If RUN(I) is called, t and output return the values for each
        %   time step of the internal ode solver, which makes than larger
        %   than simply one scalar t and one output vector for time t.
        %
        %   See also RESET, SETROTORRUNNING, SETROTORSTATUS, CLEARLOG
        
            simInput = [];
            simTime = [];
            % verify inputs types and their sizes
            if size(input,1)==obj.numberOfRotors_ && isnumeric(input)
                % Verify if model can be simulated considering configured
                % variables
                if obj.canRun()
                    % If yes, verify if its already running
                    if obj.isRunning_ == false
                        if obj.canStart()
                            % if not running already, copy initial states to previous states variable
                            obj.previousState_ = obj.initialState_; 
                            obj.previousState_.time = 0;
                            obj.isRunning_ = true;
                            firstRun = true;
                        else
                            error('Cannot start simulation. Configure initial states')
                        end
                    else
                        firstRun = false;
                    end
                    % break input and time vectors into simulation step sizes
                    simInput(:,1) = obj.previousState_.input;
                    simTime(1) = obj.previousState_.time;
                    if length(varargin)>=1
                        if isvector(varargin{1}) && isnumeric(varargin{1}) && ~isempty(varargin{1})
                            if varargin{1}>obj.previousState_.time
                                time = varargin{1};
                            else
                                error('Time values must be greater than 0 or previous run time')
                            end
                        else
                            error('Second argument must be a numeric vector of time values.')
                        end
                        aux = obj.timeStep_ + simTime(1);
                        for i=1:size(input,2)
                            if i<=length(time)
                                while(aux<time(i))
                                    simTime = [simTime aux];
                                    simInput = [simInput simInput(:,end)];
                                    aux = aux + obj.timeStep_;
                                end
                                simTime = [simTime aux];
                                simInput = [simInput input(:,i)];
                                aux = aux + obj.timeStep_;
                            else
                                simTime = [simTime aux];
                                simInput = [simInput input(:,i)];
                                aux = aux + obj.timeStep_;
                            end
                        end
                        if length(time)>size(input,2)
                            aux = (simTime(end)+obj.timeStep_):obj.timeStep_:time(end);
                            simTime = [simTime aux];
                            simInput = [simInput simInput(:,end)*ones(1,length(aux))];
                        end
                    else
                        for i=1:size(input,2)
                            simInput(:,i+1) = input(:,i);
                            simTime(i+1) = i*obj.timeStep_ + simTime(1);
                        end
                    end
                    % Run simulation
                    switch obj.simEffects_{2}
                        case 'solver ode45'
                            if firstRun==true
                                y0 = [obj.previousState_.position; obj.previousState_.attitude; obj.previousState_.velocity; obj.previousState_.angularVelocity];
                                switch obj.simEffects_{1}
                                    case 'motor dynamics on'
                                        y0 = [y0; [obj.previousState_.rotor(:).speed]'];
                                    case 'motor dynamics tf on'
                                        y0 = [y0; [obj.previousState_.rotor(:).speed]'; [obj.previousState_.rotor(:).acceleration]'];
                                    otherwise
                                        % does nothing
                                end                    
                                obj.solver_ = ode45(@(t,y) obj.model(t,y,simTime,simInput), [simTime(1) simTime(end)], y0, obj.opts_);
                            else
                                obj.solver_ = odextend(obj.solver_, @(t,y) obj.model(t,y,simTime,simInput), simTime(end));
                            end
                            t = simTime';
                            output = deval(obj.solver_,t)';
                        case 'solver euler'
                            y0 = [obj.previousState_.position; obj.previousState_.attitude; obj.previousState_.velocity; obj.previousState_.angularVelocity];
                            switch obj.simEffects_{1}
                                    case 'motor dynamics on'
                                        y0 = [y0; [obj.previousState_.rotor(:).speed]'];
                                case 'motor dynamics tf on'
                                    y0 = [y0; [obj.previousState_.rotor(:).speed]'; [obj.previousState_.rotor(:).acceleration]'];
                                otherwise
                                    % does nothing
                            end  
                            output = zeros(size(y0,1),length(simTime));
                            output(:,1) = y0;
                            for it=1:length(simTime)-1
                                dydt = obj.model(simTime(it),output(:,it),simTime,simInput);
                                output(:,it+1) = output(:,it) + obj.timeStep_*dydt;
                                output(4:7,it+1) = output(4:7,it+1)/norm(output(4:7,it+1));
                            end
                            t = simTime';
                            output = output';
                        otherwise
                            error('No solver configured')
                    end
                    obj.previousState_.position = output(end,1:3)';
                    obj.previousState_.attitude = output(end,4:7)';
                    obj.previousState_.velocity = output(end,8:10)';
                    obj.previousState_.angularVelocity = output(end,11:13)';
                    obj.previousState_.time = t(end);
                    obj.previousState_.input = simInput(:,end);
                    dydt = obj.dydtAux_;
                    switch obj.simEffects_{1}
                        case 'motor dynamics on'
                            clear aux
                            aux = output(end,14:(13+obj.numberOfRotors_))';
                            aux = num2cell(aux);
                            [obj.previousState_.rotor(:).speed] = aux{:};
                            clear aux
                            aux = dydt(14:(13+obj.numberOfRotors_))';
                            aux = num2cell(aux);
                            [obj.previousState_.rotor(:).acceleration] = aux{:};
                        case 'motor dynamics tf on'
                            clear aux
                            aux = output(end,14:(13+obj.numberOfRotors_))';
                            aux = num2cell(aux);
                            [obj.previousState_.rotor(:).speed] = aux{:};
                            clear aux
                            aux = output(end,(14+obj.numberOfRotors_):(13+2*obj.numberOfRotors_))';
                            aux = num2cell(aux);
                            [obj.previousState_.rotor(:).acceleration] = aux{:};
                        otherwise
                            if firstRun == true
                               obj.rotorSpeedsAux_(:,1) = []; 
                            end
                    end
                    %y0 = [obj.previousState_.position; obj.previousState_.attitude; obj.previousState_.velocity; obj.previousState_.angularVelocity];
                    %switch obj.simEffects_{1}
                    %    case 'motor dynamics tf on'
                    %        y0 = [y0; [obj.previousState_.rotor(:).speed]'; [obj.previousState_.rotor(:).acceleration]'];
                    %    otherwise
                            % does nothing
                    %end
                    %dydt = obj.model(obj.previousState_.time,y0,simTime,simInput);
                    obj.previousState_.acceleration = dydt(8:10);
                    obj.previousState_.angularAcceleration = dydt(11:13);
                    if firstRun == false
                        t(1) = [];
                        output(1,:) = [];
                    end
                    if obj.isLogging_ == true
                        previousLength = length(obj.log_.time);
                        if size(simInput,2)==2
                            simInput = interp1(simTime, simInput',t);
                            sizeaux = size(simInput',1);
                            sizelog = size(obj.log_.input,1);
                            if sizeaux >= sizelog
                                obj.log_.input = [[obj.log_.input; zeros(sizeaux-sizelog,size(obj.log_.input,2))],simInput'];
                            else
                                obj.log_.input = [obj.log_.input, [simInput'; zeros(sizelog-sizeaux,size(simInput',2))]];
                            end
                        else
                            if firstRun==false
                                sizeaux = size(simInput(:,2:end),1);
                                sizelog = size(obj.log_.input,1);
                                if sizeaux >= sizelog
                                    obj.log_.input = [[obj.log_.input; zeros(sizeaux-sizelog,size(obj.log_.input,2))],simInput(:,2:end)];
                                else
                                    obj.log_.input = [obj.log_.input, [simInput(:,2:end); zeros(sizelog-sizeaux,size(simInput(:,2:end),2))]];
                                end
                            else
                                sizeaux = size(simInput,1);
                                sizelog = size(obj.log_.input,1);
                                if sizeaux >= sizelog
                                    obj.log_.input = [[obj.log_.input; zeros(sizeaux-sizelog,size(obj.log_.input,2))],simInput];
                                else
                                    obj.log_.input = [obj.log_.input, [simInput; zeros(sizelog-sizeaux,size(simInput,2))]];
                                end
                            end
                        end
                        obj.log_.time = [obj.log_.time,t'];
                        obj.log_.position = [obj.log_.position,output(:,1:3)'];
                        obj.log_.attitude = [obj.log_.attitude,output(:,4:7)'];
                        obj.log_.velocity = [obj.log_.velocity,output(:,8:10)'];
                        obj.log_.angularVelocity = [obj.log_.angularVelocity,output(:,11:13)'];
%                         [uniqueTimes,uniqueIndices,~] = unique(obj.spTimeAux_');
%                         sp = interp1(uniqueTimes,obj.setPointsAux_(:,uniqueIndices)',t);
                        sp = [];
                        rspeeds = [];
                        raccs = [];
%                         disp(t)
%                         disp(obj.spTimeAux_)
%                         pause
                        for i=t'
                            spIndex = find(i>=obj.spTimeAux_,1,'last');
                            sp = [sp;obj.setPointsAux_(:,spIndex)'];
                            if strcmp(obj.simEffects_{1},'motor dynamics off')
                                rspeeds = [rspeeds;obj.rotorSpeedsAux_(:,spIndex)'];
                            end
                            if strcmp(obj.simEffects_{1},'motor dynamics on')
                                raccs = [raccs;obj.rotorAccAux_(:,spIndex)'];
                            end
                        end
                        power = zeros(1,length(t));
                        for i=1:obj.numberOfRotors_
                            if i>length(obj.log_.rotor)
                                obj.log_.rotor(i).speed = zeros(1,previousLength);
                                obj.log_.rotor(i).setPoint = zeros(1,previousLength);
                                obj.log_.rotor(i).torque = zeros(1,previousLength);
                                if strcmp(obj.simEffects_{1},'motor dynamics tf on') || ...
                                    strcmp(obj.simEffects_{1},'motor dynamics on')
                                    obj.log_.rotor(i).acceleration = zeros(1,previousLength);
                                end
                            end
                            obj.log_.rotor(i).setPoint = [obj.log_.rotor(i).setPoint,sp(:,i)'];
                            switch obj.simEffects_{1}
                                case 'motor dynamics on'
                                    speed = output(:,13+i)';
                                    obj.log_.rotor(i).speed = [obj.log_.rotor(i).speed,speed];
                                    accel = raccs(:,i)';
                                    obj.log_.rotor(i).acceleration = [obj.log_.rotor(i).acceleration,accel];
                                    torque = obj.rotorDragCoeff(i)*(speed.*abs(speed))+obj.rotorInertia(i)*accel;
                                    obj.log_.rotor(i).torque = [obj.log_.rotor(i).torque,torque];
                                    power = power + torque.*speed;
                                case 'motor dynamics tf on'
                                    speed = output(:,13+i)';
                                    obj.log_.rotor(i).speed = [obj.log_.rotor(i).speed,speed];
                                    accel = output(:,13+obj.numberOfRotors_+i)';
                                    obj.log_.rotor(i).acceleration = [obj.log_.rotor(i).acceleration,accel];
                                    torque = obj.rotorDragCoeff(i)*(speed.*abs(speed))+obj.rotorInertia(i)*accel;
                                    obj.log_.rotor(i).torque = [obj.log_.rotor(i).torque,torque];
                                    power = power + torque.*speed;
                                otherwise
                                    speed = rspeeds(:,i)';
                                    obj.log_.rotor(i).speed = [obj.log_.rotor(i).speed,rspeeds(:,i)'];
                                    torque = obj.rotorDragCoeff(i)*(speed.*abs(speed));
                                    obj.log_.rotor(i).torque = [obj.log_.rotor(i).torque,torque];
                                    power = power + torque.*speed;
                            end
                        end
                        obj.log_.power = [obj.log_.power, power];
                        %obj.setPointsAux_ = [];
                        %obj.rotorSpeedsAux_ = [];
                        %obj.spTimeAux_ = [];
                        for i=(obj.numberOfRotors_+1):length(obj.log_.rotor)
                            obj.log_.rotor(i).speed = [obj.log_.rotor(i).speed,zeros(1,length(t))];
                            obj.log_.rotor(i).setPoint = [obj.log_.rotor(i).setPoint,zeros(1,length(t))];
                            obj.log_.rotor(i).torque = [obj.log_.rotor(i).torque,zeros(1,length(t))];
                            if strcmp(obj.simEffects_{1},'motor dynamics tf on') || strcmp(obj.simEffects_{1},'motor dynamics on')
                                obj.log_.rotor(i).acceleration = [obj.log_.rotor(i).acceleration,zeros(1,length(t))];
                            end
                        end
                    end      
                else
                    error('Cannot run. Model not configured correctly.')
                    output = [];
                    t = [];
                end      
            else
                error('Input must be a numeric array of N rows, where N is the current number of rotors!')
                output = [];
                t = [];
            end
        end  
        function reset(obj)
        %RESET Resets simulation.
        %
        %   Resets simulation to initial states and time 0. Does not clear
        %   log history.
        %   
        %   See also RUN, STOPLOGGING, CLEARLOG, SETROTORRUNNING,
        %   SETROTORSTATUS
            obj.previousState_ = obj.initialState_;                            
            obj.previousState_.time = 0;
            obj.isRunning_ = false;
            obj.setRotorOK(1:obj.numberOfRotors_);
            obj.setPointsAux_ = [];
            obj.rotorSpeedsAux_ = [];
            obj.spTimeAux_ = [];
        end
        function setSimEffects(obj, varargin)
        %SIMEFFECTS Controls which simulation effects will run.
        %
        %   SIMEFFECTS() Does nothing.
        %
        %   SIMEFFECTS(effectType, effectArg) Selects simulation effects
        %   specified by effectType.
        %   effectType must be a string specifying the type of effect to be
        %   considered or not for simulation.
        %   Types are:
        %       - 'motor dynamics on': Simulates motor dynamics based on
        %       brushless motor model from "Permanent Magnet Motor
        %       Technology: Design and Applications", Jacek F. Gieras.
        %       Multicopter inputs become motor voltages.
        %       No effectArg needed.
        %       - 'motor dynamics tf on': Simulates motor dynamics as a
        %       transfer function between input reference and motor speed.
        %       Multicopter inputs become rotor speeds.
        %       No effectArg needed.
        %       - 'motor dynamics off': Does not simulate motor dynamics,
        %       making mapping inputs directly to rotor speeds. Multicopter
        %       inputs become rotor speeds.
        %       No effectArg needed.
        %       - 'solver ode45': Uses MATLAB's ode45 solver for
        %       simulation.
        %       No effectArg needed.
        %       - 'solver euler': Uses Euler's method for diff equation
        %       solving (x(t+h) = dydt(t)*h+x(t)).
        %       No effectArg needed.
        %   
        %   effectType and effectArg can be
        %   defined an arbitrary number of times in function call as long as 
        %   effectArg follows effectType when an argument is necessary for the 
        %   type of effect specified.
        %   
        %   See also SETROTORTF, ROTORSTATUS, RUN, RESET, STOPLOGGING,
        %   CLEARLOG.
     
        if length(varargin)>=1 && ~isempty(varargin{1})
            i = 1;
            while i<=length(varargin)
                switch varargin{i}
                    case 'motor dynamics tf on'
                        obj.simEffects_{1} = varargin{i};
                    case 'motor dynamics on'
                        obj.simEffects_{1} = varargin{i};
                    case 'motor dynamics off'                 
                        obj.simEffects_{1} = varargin{i};
                    case 'solver ode45'
                        obj.simEffects_{2} = varargin{i};
                    case 'solver euler'
                        obj.simEffects_{2} = varargin{i};
                    otherwise
                        warning('Simulation configuration unknown.');
                end
                i = i+1;
            end
        end
        end
        function simEffectsValue = simEffects(obj)
        %SIMEFFECTS Returns the current simulation effects configured.
        %
        %   S = SIMEFFECTS() Returns the simulation effects considered for
        %   the current simulation.
        %
        %   S is a cell of strings. 
        %
        %   Check setSimEffects() for more information.
        %
        %   See also SETSIMEFFECTS.
        
            simEffectsValue = obj.simEffects_;
        end       
        function setRotorStatus(obj, rotorID, varargin)
        %SETROTORSTATUS Simulates rotor fail or rotor normal operation.
        %
        %   SETROTORSTATUS(rotorID) Does nothing.
        %
        %   SETROTORSTATUS(rotorID, statusType, statusArg) Specifies rotor status as
        %   statusType with arguments statusArg. statusType and statusArg can be
        %   defined an arbitrary number of times in function call as long as 
        %   statusArg follows statusType when an argument is necessary for the 
        %   type of status specified.
        %
        %   rotorID may be scalar or an arbitrary vector of rotor IDs. 
        %
        %   statusType must be a string specifying the type of fail or operation. 
        %   Types are:
        %       - 'stuck': Rotor speeds are set to zero after a period
        %       specified by statusArg. statusArg must not be zero.
        %       - 'free': Rotor speeds respond freely acording to rotor
        %       transfer function.
        %       - 'non-responding': Freezes rotor set points to last set
        %       points. No argument needed.
        %       - 'responding': Rotors set points are updated normally
        %       according to inputs.
        %       - 'motor loss': Rotors settling time and steady state error
        %       are increased based on efficiency specified by statusArg. a
        %       value of 0 means that rotor speeds do not follow set
        %       points, while a value of 1 corresponds to normal operation.
        %       - 'motor ok': Sets rotor efficiencies to 1.
        %       - 'prop loss': Propeller inertia, lift and drag
        %       coefficients are reduced according to propeller efficiency
        %       specified in statusArg. A value of 0 means that the
        %       propeller produces no torque and lift, while 1 corresponds
        %       to normal operation.
        %       - 'prop ok': Sets propeller efficiencies to 1.
        %
        %   statusArg may be a scalar value or an array specifying the
        %   status argument for each rotor from rotorID.
        %
        %   Example:
        %   SETROTORSTATUS([1 2 5],'stuck',[0.01 0.05 0.02]) sets rotors 1,
        %   2 and 5 to speed 0 in 0.01, 0.05 and 0.02 seconds,
        %   respectively.
        %   
        %   See also SETROTOROK, ROTORSTATUS, RUN, RESET, STOPLOGGING,
        %   CLEARLOG.
     
        if length(varargin)>=1 && ~isempty(varargin{1})
            i = 1;
            while i<=length(varargin)
                switch varargin{i}
                    case 'stuck'
                        if length(varargin)>=(i+1) && ~isempty(varargin{i+1}) && isnumeric(varargin{i+1})
                            for j=1:length(rotorID)
                                obj.rotor_(rotorID(j)).status{1} = varargin{i};
                            end
                            i = i+1;
                            period = num2cell(varargin{i});
                            [obj.rotor_(rotorID).stuckTransitionPeriod] = period{:};
                        else
                            warning('Missing argument for motor stuck fault');
                        end
                    case 'free'                 
                        for j=1:length(rotorID)
                            obj.rotor_(rotorID(j)).status{1} = varargin{i};
                        end
                    case 'non-responding'               
                        for j=1:length(rotorID)
                            obj.rotor_(rotorID(j)).status{2} = varargin{i};
                        end
                    case 'responding'               
                        for j=1:length(rotorID)
                            obj.rotor_(rotorID(j)).status{2} = varargin{i};
                        end
                    case 'motor loss' 
                        if length(varargin)>=(i+1) && ~isempty(varargin{i+1}) && isnumeric(varargin{i+1})
                            for j=1:length(rotorID)
                                obj.rotor_(rotorID(j)).status{3} = varargin{i};
                            end
                            i = i+1;
                            efficiency = num2cell(varargin{i});
                            [obj.rotor_(rotorID).motorEfficiency] = efficiency{:};
                        else
                            warning('Missing argument for motor efficiency');
                        end
                    case 'motor ok'            
                        for j=1:length(rotorID)
                            obj.rotor_(rotorID(j)).status{3} = varargin{i};
                        end
                        [obj.rotor_(rotorID).motorEfficiency] = deal(1);
                    case 'prop loss'
                        if length(varargin)>=(i+1) && ~isempty(varargin{i+1}) && isnumeric(varargin{i+1})
                            for j=1:length(rotorID)
                                obj.rotor_(rotorID(j)).status{4} = varargin{i};
                            end
                            i = i+1;
                            efficiency = num2cell(varargin{i});
                            [obj.rotor_(rotorID).propEfficiency] = efficiency{:};
                        else
                            warning('Missing argument for propeller efficiency');
                        end
                    case 'prop ok'            
                        for j=1:length(rotorID)
                            obj.rotor_(rotorID(j)).status{4} = varargin{i};
                        end
                        [obj.rotor_(rotorID).propEfficiency] = deal(1);                        
                    otherwise
                        warning('Rotor fail type is unknown');
                end
                i = i+1;
            end
        end
        end
        function rotorStatusValue = rotorStatus(obj, rotorID)
        %ROTORSTATUS Returns the status of the specified rotor.
        %
        %   S = ROTORSTATUS(rotorID) Returns the status of the specified rotor.
        %   rotorID specifies which rotor to return the status of.
        %   rotorID can be a single value or an array of IDs.
        %   S is a cell of size 1xN, where N = length(rotorID) and each
        %   cell element contains the status associated with 
        %   the rotor specified by the respective rotorID. 
        %
        %   Rotor status can be 'stuck' or 'free';'non-responding' or
        %   'responding'; 'motor loss' or 'motor ok'; and 'prop loss' or
        %   'prop ok'. Check setRotorStatus() for more information.
        %
        %   See also SETROTORSTATUS, SETROTOROK, ROTOR, ROTORPOSITION, 
        %   ROTORORIENTATION, ROTORINERTIA, ROTORLIFTCOEFF, ROTORDRAGCOEFF,
        %   ROTOREFFICIENCY, ROTORMAXSPEED.
        
            rotorStatusValue = {obj.rotor_(rotorID).status};
        end       
        function setRotorOK(obj, rotorID)
        %SETROTOROK Sets the rotor to normal functioning.
        %
        %   SETROTOROK(rotorID) Allows rotor to run normally and sets
        %   rotor efficiency as 100%. rotorID may be scalar or an arbitrary
        %   vector of IDs.
        %   
        %   See also RUN, RESET, STOPLOGGING, CLEARLOG, SETROTORSTATUS
    
        obj.setRotorStatus(rotorID,'free','responding','motor ok','prop ok');
        end      
        
        %%% Utils functions        
        newCopy                 = copy(obj);
        
        function eulerAngles = toEuler(obj, quaternion)
        %TOEULER Converts a quaternion to aeronautical euler angles
        %
        %   A = TOEULER(Q) Converts a quaternion Q to aeronautical euler
        %   angles A. Q may be a numeric vector of length 4 or a numeric
        %   array of 4 rows.
        %   If Q is a vector, A is a numeric vector of length 3 where A(1), 
        %   A(2) and A(3) are row, pitch and yaw, respectively.
        %   If Q is a matrix, A is a numeric matrix with 4 rows and number 
        %   of columns equal to number of columns of Q, where A(1,:), 
        %   A(2,:) and A(3,:) are row, pitch and yaw, respectively.
        %
        %   See also TOQUATERNION, MATRIXATOB, MATRIXBTOA
            %quaternion = quaternion/norm(quaternion);
            if isvector(quaternion) && isnumeric(quaternion) && length(quaternion)==4
                eulerAngles(1) = atan2(2*quaternion(3)*quaternion(4)+2*quaternion(1)*quaternion(2),quaternion(4)^2-quaternion(3)^2-quaternion(2)^2+quaternion(1)^2);
                eulerAngles(2) = -asin(2*quaternion(2)*quaternion(4)-2*quaternion(1)*quaternion(3));
                eulerAngles(3) = atan2(2*quaternion(2)*quaternion(3)+2*quaternion(1)*quaternion(4),quaternion(2)^2+quaternion(1)^2-quaternion(4)^2-quaternion(3)^2);
            elseif isnumeric(quaternion) && size(quaternion,1)==4
                for i=1:size(quaternion,2)
                    eulerAngles(1,i) = atan2(2*quaternion(3,i)*quaternion(4,i)+2*quaternion(1,i)*quaternion(2,i),quaternion(4,i)^2-quaternion(3,i)^2-quaternion(2,i)^2+quaternion(1,i)^2);
                    eulerAngles(2,i) = -asin(2*quaternion(2,i)*quaternion(4,i)-2*quaternion(1,i)*quaternion(3,i));
                    eulerAngles(3,i) = atan2(2*quaternion(2,i)*quaternion(3,i)+2*quaternion(1,i)*quaternion(4,i),quaternion(2,i)^2+quaternion(1,i)^2-quaternion(4,i)^2-quaternion(3,i)^2);
                end
            else
                error('Quaternion must be a numeric vector of length 4 or numeric array of 4 rows.')
            end
            eulerAngles = real(eulerAngles);
        end        
        function quaternion = toQuaternion(obj, eulerAngles)
        %TOQUATERNION Converts a set o Euler angles to quaternion
        %representation.
        %
        %   Q = TOQUATERNION(A) Converts a set of Euler angles to
        %   quaternion representation.
        %   A may be a numeric vector of length 3 where A(1), A(2) and 
        %   A(3) are row, pitch and yaw, respectively, or a numeric array
        %   of 3 rows where A(1,:), A(2,:) and A(3,:) are row, pitch and 
        %   yaw, respectively.
        %   If A is a vector, Q is a numeric vector of length 4.
        %   If A is a matrix, Q is a matrix with 3 rows and number of
        %   columns equal to number of columns of A, where each column
        %   represents a quaternion relative to each column euler
        %   angle vector.
        %
        %   See also TOEULER, MATRIXATOB, MATRIXBTOA
        
            if isvector(eulerAngles) && isnumeric(eulerAngles) && length(eulerAngles)==3
                quaternion(1) =  cos(eulerAngles(1)/2)*cos(eulerAngles(2)/2)*cos(eulerAngles(3)/2)+sin(eulerAngles(1)/2)*sin(eulerAngles(2)/2)*sin(eulerAngles(3)/2);
                quaternion(2) = -cos(eulerAngles(1)/2)*sin(eulerAngles(2)/2)*sin(eulerAngles(3)/2)+cos(eulerAngles(2)/2)*cos(eulerAngles(3)/2)*sin(eulerAngles(1)/2);
                quaternion(3) =  cos(eulerAngles(1)/2)*cos(eulerAngles(3)/2)*sin(eulerAngles(2)/2)+sin(eulerAngles(1)/2)*cos(eulerAngles(2)/2)*sin(eulerAngles(3)/2);
                quaternion(4) =  cos(eulerAngles(1)/2)*cos(eulerAngles(2)/2)*sin(eulerAngles(3)/2)-sin(eulerAngles(1)/2)*cos(eulerAngles(3)/2)*sin(eulerAngles(2)/2);
            elseif isnumeric(eulerAngles) && size(eulerAngles,1)==3
                for i=1:size(eulerAngles,2)
                    quaternion(1,i) =  cos(eulerAngles(1,i)/2)*cos(eulerAngles(2,i)/2)*cos(eulerAngles(3,i)/2)+sin(eulerAngles(1,i)/2)*sin(eulerAngles(2,i)/2)*sin(eulerAngles(3,i)/2);
                    quaternion(2,i) = -cos(eulerAngles(1,i)/2)*sin(eulerAngles(2,i)/2)*sin(eulerAngles(3,i)/2)+cos(eulerAngles(2,i)/2)*cos(eulerAngles(3,i)/2)*sin(eulerAngles(1,i)/2);
                    quaternion(3,i) =  cos(eulerAngles(1,i)/2)*cos(eulerAngles(3,i)/2)*sin(eulerAngles(2,i)/2)+sin(eulerAngles(1,i)/2)*cos(eulerAngles(2,i)/2)*sin(eulerAngles(3,i)/2);
                    quaternion(4,i) =  cos(eulerAngles(1,i)/2)*cos(eulerAngles(2,i)/2)*sin(eulerAngles(3,i)/2)-sin(eulerAngles(1,i)/2)*cos(eulerAngles(3,i)/2)*sin(eulerAngles(2,i)/2);
                end
            else
                error('Euler Angles must be a numeric vector of length 3 or numeric array with 3 rows.')
            end
        end         
        function transformationBA = matrixBodyToAbsolute(obj)
        %MATRIXBODYTOABSOLUTE Current transformation matrix body to absolute
        %
        %   Q = MATRIXBODYTOABSOLUTE() Returns current transformation
        %   matrix from body reference frame to absolute reference frame,
        %   considering current body attitude. 
        %   Q is a numeric symmetric transformation matrix.
        %
        %   See also TOEULER, TOQUATERNION, MATRIXBTOA, MATRIXATOB
        
            quaternion = obj.previousState_.attitude;
            transformationBA = obj.matrixBtoA(quaternion);
        end        
        function transformationAB = matrixAbsoluteToBody(obj)
        %MATRIXABSOLUTETOBODY Current transformation matrix absolute to
        %body
        %
        %   Q = MATRIXABSOLUTETOBODY() Returns current transformation
        %   matrix from absolute reference frame to body reference frame,
        %   considering current body attitude. 
        %   Q is a numeric symmetric transformation matrix.
        %
        %   See also TOEULER, TOQUATERNION, MATRIXBTOA, MATRIXATOB
        
            quaternion = obj.previousState_.attitude;
            transformationAB = obj.matrixAtoB(quaternion);
        end        
        function transformationAB = matrixAtoB(obj, quaternion)
        %MATRIXATOB Transformation matrix absolute to frame B
        %
        %   Q = MATRIXATOB(q) Returns a transformation matrix from absolute 
        %   reference frame to a reference frame represented by quaternion 
        %   q. q must be a numeric vector of length 4.
        %   Q is a numeric symmetric transformation matrix.
        %
        %   See also TOEULER, TOQUATERNION, MATRIXBTOA,
        %   MATRIXABSOLUTETOBODY
        
            if isvector(quaternion) && isnumeric(quaternion) && length(quaternion)==4
                transformationAB = obj.matrixBtoA(quaternion);
                transformationAB = transformationAB';
            else
                error('Quaternion must be a numeric vector of length 4')
            end
        end       
        function transformationBA = matrixBtoA(obj, quaternion)
        %MATRIXBTOA Transformation matrix frame B to absolute
        %
        %   Q = MATRIXBTOA(q) Returns a transformation matrix from a
        %   reference frame represented by quaternion q, to the absolute
        %   reference frame. q must be a numeric vector of length 4.
        %   Q is a numeric symmetric transformation matrix.
        %
        %   See also TOEULER, TOQUATERNION, MATRIXATOB,
        %   MATRIXBODYTOABSOLUTE
            transformationBA = zeros(3,3);
            if isvector(quaternion) && isnumeric(quaternion) && length(quaternion)==4
                transformationBA(1,1) = quaternion(1)^2+quaternion(2)^2-quaternion(3)^2-quaternion(4)^2;
                transformationBA(1,2) = 2*(quaternion(2)*quaternion(3)-quaternion(1)*quaternion(4));
                transformationBA(1,3) = 2*(quaternion(1)*quaternion(3)+quaternion(2)*quaternion(4));
                transformationBA(2,1) = 2*(quaternion(2)*quaternion(3)+quaternion(1)*quaternion(4));
                transformationBA(2,2) = quaternion(1)^2-quaternion(2)^2+quaternion(3)^2-quaternion(4)^2;
                transformationBA(2,3) = 2*(quaternion(3)*quaternion(4)-quaternion(1)*quaternion(2));
                transformationBA(3,1) = 2*(quaternion(2)*quaternion(4)-quaternion(1)*quaternion(3));
                transformationBA(3,2) = 2*(quaternion(3)*quaternion(4)+quaternion(1)*quaternion(2));
                transformationBA(3,3) = quaternion(1)^2-quaternion(2)^2-quaternion(3)^2+quaternion(4)^2;
            else
                error('Quaternion must be a numeric vector of length 4')
            end
        end
        
    end
    
    methods (Access = protected)        
        dydt = model(obj,t,y,simTime,simInput)
        function result = canRun(obj)
            result = true;
            if isempty(obj.mass)
                error('Cannot run. Specify aircraft mass.')
                result = false;
            end
            if isempty(obj.inertiaTensor_)
                error('Cannot run. Specify aircraft inertia tensor.')
                result = false;
            end
            if isempty(obj.translationalFriction_)
                error('Cannot run. Specify aircraft translational drag.')
                result = false;
            end
            if (obj.timeStep_ <= 0.0)||isempty(obj.timeStep_)
                error('Cannot run. Specify simulation time step.')
                result = false;
            end            
            if isempty(obj.isLogging_)
                error('Cannot run. Specify whether simulation should be logged or not.')
                result = false;
            end
            if obj.numberOfRotors_>0 && isempty(obj.rotor_)
                error('Cannot run. Object specifies %d number of rotors but no rotor definition was found.',obj.numberOfRotors_)
                result = false;
            elseif obj.numberOfRotors_>0
                for i=1:obj.numberOfRotors_
                    if isempty(obj.rotor_(i).position)
                        error('Cannot run. Specify rotor %d position.',i)
                        result = false;
                    end
                    if isempty(obj.rotor_(i).orientation)
                        error('Cannot run. Specify rotor %d orientation.',i)
                        result = false;
                    end
                    if isempty(obj.rotor_(i).inertia)
                        error('Cannot run. Specify rotor %d inertia.',i)
                        result = false;
                    end
                    if isempty(obj.rotor_(i).liftCoeff)
                        error('Cannot run. Specify rotor %d lift coefficient.',i)
                        result = false;
                    end
                    if isempty(obj.rotor_(i).dragCoeff)
                        error('Cannot run. Specify rotor %d drag coefficient.',i)
                        result = false;
                    end
                    if isempty(obj.rotor_(i).maxSpeed)
                        error('Cannot run. Specify rotor %d maximum speed.',i)
                        result = false;
                    end
                    if isempty(obj.rotor_(i).minSpeed)
                        error('Cannot run. Specify rotor %d minimum speed.',i)
                        result = false;
                    end
                    if isempty(obj.rotor_(i).transferFunction) && strcmp(obj.simEffects_{1},'motor dynamics tf on')
                        error('Cannot run. Specify rotor %d transfer function.',i)
                        result = false;
                    end
                end
            end
            
            if obj.numberOfRotors_<1
                warning('No rotors configured for aircraft.')
            end
        end       
        function result = canStart(obj)
            result = true;
            if isempty(obj.initialState_)
                error('Cannot start simulation. Specify initial state variables.')
                result = false;
            else
                if isempty(obj.initialState_.position)
                    error('Cannot start simulation. Specify initial position.')
                    result = false;
                end
                if isempty(obj.initialState_.attitude)
                    error('Cannot start simulation. Specify initial attitude.')
                    result = false;
                end
                if isempty(obj.initialState_.velocity)
                    error('Cannot start simulation. Specify initial velocity.')
                    result = false;
                end
                if isempty(obj.initialState_.angularVelocity)
                    error('Cannot start simulation. Specify initial angular velocity.')
                    result = false;
                end
                if obj.numberOfRotors_>0 && isempty(obj.initialState_.rotor)                     
                    error('Cannot start simulation. Specify initial states for the rotors.')
                    result = false;
                elseif obj.numberOfRotors_>0
                    for i=1:obj.numberOfRotors_
                        if isempty(obj.initialState_.rotor(i).speed)
                            error('Cannot start simulation. Specify initial rotor speed for rotor %d.',i)
                            result = false;
                        end
                        if isempty(obj.initialState_.rotor(i).acceleration) && strcmp(obj.simEffects_{1},'motor dynamics tf on')
                            error('Cannot start simulation. Specify initial rotor acceleration for rotor %d.',i)
                            result = false;
                        end
                    end
                end                
                if obj.numberOfRotors_>0 && isempty(obj.initialState_.input)                
                    error('Cannot start simulation. Specify initial inputs.')
                    result = false;
                elseif obj.numberOfRotors_>0 && length(obj.initialState_.input)~=obj.numberOfRotors_              
                    error('Cannot start simulation. Specify initial inputs for all rotors.')
                    result = false;
                end
            end
        end
    end
end

