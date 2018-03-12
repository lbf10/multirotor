classdef multicopter < handle
    %MULTICOPTER Simulation class for a multicopter of arbitrary number of
    %rotors. Model version 1.0 
    %
    %   University of São Paulo - USP
    %   Author: Leonardo Borges Farçoni                       
    %   e-mail: leonardo.farconi@gmail.com                    
    %   Professor Advisor: Marco H. Terra and Roberto Inoue   
    %   E-mail: terra@sc.usp.br and rsinoue@ufscar.br         
    %   Date: June 23rd, 2017                                 
    %   Revision 1:                                           
    %
    %   Model and class considerarions/hypothesis:
    %       - Body frame centered at CG/CM
    %       - Arbitrary number of rotors
    %       - Arbitrary rotor positions
    %       - Arbitrary rotor orientations
    %       - No blade-flapping
    %       - Quaternion modelling allowing any kind of rotations
    %       - Fixed CG/CM
    %       - Only translational aerodynamic drag
    %       - Lift and drag proportional to square of rotor speed 
    %       - Rotors variables can be changed during run time, but they are
    %       considered as constants and not dynamic variables in the model.
    %       - Euler angles are [row, pitch, yaw] always in this order,
    %       assuming a (1,2,3) rotation sequence, or in other words, the
    %       nautical and aeronautical convention. Reference: Representing 
    %       Attitude: Euler Angles, Unit Quaternions, and Rotation Vectors,
    %       J. Diebel, 2006.
    %       - No rotor dynamics considered
    %       - Inputs are rotor speeds
    %       - Rotors create lift only to the direction specified by the
    %       rotor orientation (rotors are not revertible)
    %
    %   This class is simulates any kind of multicopter under
    %   the considerations above. Its intended use is to evaluate control
    %   and estimation algorithms. See example on run() function
    %   description.
    %
    %   To create a multicopter object, call the multicopter constructor,
    %   add the necessary number of rotors and set parameters using set
    %   functions. The aircraft dynamics and input maping are defined by 
    %   the private function model().
    %   
    %   Parameters can be set and get individually or in batches.
    %   Use "doc multicopter" for methods listing and more information.
    %   
    %   See also SETMASS, SETINERTIA, SETFRICTION, ADDROTOR, SETTIMESTEP,
    %   SETINITIALSTATE, STARTLOGGING, RUN, RESET.
    
    properties (Access = private)
        % Aircraft parameters
        mass_                   % Aircraft's total mass
        inertiaTensor_          % 3x3 matrix with inertia products in relation to body reference frame
        translationalFriction_  % 3x3 diagonal matrix with friction in each direction in relation to body reference frame
        numberOfRotors_         % scalar for number of either active or fail rotors
        rotor_                  % struct containing all rotor parameters and states - position and orientation in relation to body frame, inertia in relation to rotation axis, thrust coefficient, torque coefficient, status flag, efficiency, speed
        
        %Simulation parameters
        timeStep_        % scalar for simulation time step
        initialState_    % struct containing the initial state (state is comprised by the aircraft's states and rotor states in case )
        previousState_   % struct containing the previous state, previous input and previous time
        log_             % struct of arrays containing all state, input and time values from simulation start if the class is logging
        isLogging_       % flag that indicates whether simulation should be logged or not   
        isRunning_       % flag that indicates whether simulation is running or not
    end
    
    methods        
        %%% Constructor
        function obj = multicopter(N)
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
        
            if nargin == 1
                if isnumeric(N) && N>0 && rem(N,1)==0
                    obj.numberOfRotors_ = N;
                    rotorMass = 0.468/4;
                    obj.mass_ = N*(rotorMass);                    
                    obj.translationalFriction_ = eye(3)*0.25;
                    obj.inertiaTensor_ = [0,0,0;0,0,0;0,0,0];
                    angle = 2*pi/N;
                    angles = [-angle/2,-angle/2+angle*[1:N-1]];
                    l = 0.225;
                    for i=1:N
                        obj.rotor_(i).position = l*[cos(angles(i));sin(angles(i));0];
                        obj.inertiaTensor_ = obj.inertiaTensor_+rotorMass*[obj.rotor_(i).position(2)^2, obj.rotor_(i).position(1)*obj.rotor_(i).position(2), 0;
                                                                         obj.rotor_(i).position(2)*obj.rotor_(i).position(1), obj.rotor_(i).position(1)^2, 0;
                                                                         0, 0 ,l^2];
                        obj.rotor_(i).orientation = [0;0;1];
                        obj.rotor_(i).inertia = 3.357e-5;
                        obj.rotor_(i).liftCoeff = 2.98e-6;
                        obj.rotor_(i).dragCoeff = 1.14e-7;
                        obj.rotor_(i).status = 'running';
                        obj.rotor_(i).efficiency = 1;
                        obj.rotor_(i).speedSetPoint = 0;
                        obj.rotor_(i).maxSpeed = inf;
                        obj.initialState_.rotor(i).speed = 0;
                    end                    
                    obj.timeStep_ = 0.05;
                    
                    obj.initialState_.position = [0;0;0];
                    obj.initialState_.attitude = [1;0;0;0];
                    obj.initialState_.velocity = [0;0;0];
                    obj.initialState_.angularVelocity = [0;0;0];
                    obj.initialState_.input = zeros(obj.numberOfRotors_,1);
                    
                    obj.previousState_ = obj.initialState_;      
                    obj.previousState_.acceleration = [0;0;0];   
                    obj.previousState_.angularAcceleration = [0;0;0];
                    obj.previousState_.time = 0;
                    
                    obj.isLogging_ = 1;
                    obj.log_ = [];
                    obj.isRunning_ = false;
                else
                    error('Number of rotors must be numeric, greater than  zero and integer!')
                end
            else
                obj.numberOfRotors_ = 0;
                obj.mass_ = [];
                obj.inertiaTensor_ = [];
                obj.rotor_ = [];
                obj.translationalFriction_ = [];
                obj.timeStep_ = 0.05;
                
                obj.initialState_.position = [0;0;0];
                obj.initialState_.attitude = [1;0;0;0];
                obj.initialState_.velocity = [0;0;0];
                obj.initialState_.angularVelocity = [0;0;0];
                obj.initialState_.rotor = [];
                obj.initialState_.input = [];
                
                obj.previousState_ = obj.initialState_;
                obj.previousState_.acceleration = [0;0;0];
                obj.previousState_.angularAcceleration = [0;0;0];
                obj.previousState_.time = 0;
                
                obj.isLogging_ = 1;
                obj.log_ = [];
                obj.isRunning_ = false;
            end
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
        
            if isnumeric(mass) && mass>0 && isscalar(mass)
                obj.mass_ = mass;
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
        
            if isnumeric(inertia) && (isequal(size(inertia),[1 3]) || isequal(size(inertia),[3 1]) || isequal(size(inertia),[3 3]))
                if ~isequal(size(inertia),[3 3])
                    obj.inertiaTensor_ = [inertia(1) 0 0; 0 inertia(2) 0; 0 0 inertia(3)];
                else
                    obj.inertiaTensor_ = inertia;
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
            else
                error('Translation friction matrix must be numeric and have at least 3 values!');
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
        %   ADDROTOR can be used during simulation, but in most cases it is
        %   suggested to add all rotors during initial configuration and set
        %   them as fail or running acordingly. 
        %
        %   Adding a rotor during simulation will make the log matrix to
        %   change size and waste memory, but keeping track of
        %   modifications.
        %
        %   See also REMOVEROTOR, SETROTOR, SETROTORRUNNING , SETROTORFAIL,
        %   SETROTORPOSITION, SETROTORORIENTATION, SETROTORINERTIA, 
        %   SETROTORLIFTCOEFF, SETROTORDRAGCOEFF, SETROTORMAXSPEED.
        
            rotorID = obj.numberOfRotors_+1;
            obj.numberOfRotors_ = rotorID;
            
            obj.rotor_(rotorID).position = [];
            obj.rotor_(rotorID).orientation = [];
            obj.rotor_(rotorID).inertia = [];
            obj.rotor_(rotorID).liftCoeff = [];
            obj.rotor_(rotorID).dragCoeff = [];
            obj.rotor_(rotorID).status = 'running';
            obj.rotor_(rotorID).efficiency = 1;
            obj.rotor_(rotorID).speedSetPoint = [];
            obj.rotor_(rotorID).maxSpeed = inf;
            obj.initialState_.rotor(rotorID).speed = [];
            
            obj.previousState_.input(rotorID) = 0;
            obj.previousState_.rotor(rotorID).speed = 0;
            
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
                obj.setRotorPosition(rotorID, cell2mat(varargin{1}));
            end
            if length(varargin)>=2
                obj.setRotorOrientation(rotorID, cell2mat(varargin{2}));
            end
            if length(varargin)>=3
                obj.setRotorInertia(rotorID, cell2mat(varargin{3}));
            end
            if length(varargin)>=4
                obj.setRotorLiftCoeff(rotorID, cell2mat(varargin{4}));
            end
            if length(varargin)>=5
                obj.setRotorDragCoeff(rotorID, cell2mat(varargin{5}));
            end
            if length(varargin)>=6
                obj.setRotorMaxSpeed(rotorID, cell2mat(varargin{6}));
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
            else
                error('Rotor inertia coefficients must be a vector of numeric values the same size as rotorID, which must not exceed the number of rotors in the multicopter!')
            end
        end        
        function setRotorLiftCoeff(obj, rotorID, liftCoeff)
        %SETROTORLIFTCOEFF  Configures lift coefficient for specified rotor.
        %
        %   SETROTORLIFTCOEFF(rotorID,L) Sets rotor lift coefficient to L.
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
            if isvector(liftCoeff) && isnumeric(liftCoeff) && (length(liftCoeff)==length(rotorID)) && length(rotorID)<=obj.numberOfRotors_
                liftCoeff = num2cell(liftCoeff);
                [obj.rotor_(rotorID).liftCoeff] = liftCoeff{:};
            else
                error('Lift coefficients must be a vector of numeric values the same size as rotorID, which must not exceed the number of rotors in the multicopter!')
            end
        end        
        function setRotorDragCoeff(obj, rotorID, dragCoeff)
        %SETROTORDRAGCOEFF  Configures lift coefficient for specified rotor.
        %
        %   SETROTORDRAGCOEFF(rotorID,D) Sets rotor drag coefficient to D.
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
            if isvector(dragCoeff) && isnumeric(dragCoeff) && (length(dragCoeff)==length(rotorID)) && length(rotorID)<=obj.numberOfRotors_
                dragCoeff = num2cell(dragCoeff);
                [obj.rotor_(rotorID).dragCoeff] = dragCoeff{:};
            else
                error('Drag coefficients must be a vector of numeric values the same size as rotorID, which must not exceed the number of rotors in the multicopter!')
            end
        end
        function setRotorMaxSpeed(obj, rotorID, maxSpeed)
        %SETROTORMAXSPEED  Configures maximum speed for specified rotor.
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
        
            if isscalar(timeStep) && timeStep>0
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
        function setInitialAngularVelocity(obj, angularVelocity)
        %SETINITIALANGULARVELOCITY  Sets initial multicopter velocity for simulation.
        %
        %   SETINITIALANGULARVELOCITY(V) Sets the initial angular velocity of the
        %   multicopter in relation to body frame, for simulation. 
        %   V must be a numeric vector of length 3. 
        %
        %   If needed in relation to inertial frame, use
        %   V = matrixAtoB(initialAttitude), where initial Attitude is the
        %   initial attitude in relation to absolute reference frame and V
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
        function setInitialRotorSpeeds(obj, rotorSpeeds)
        %SETINITIALROTORSPEEDS  Sets initial multicopter velocity for simulation.
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
                obj.initialState_.rotor(1:obj.numberOfRotors_).speed = aux{:};
            else
                error('Initial rotor speeds must be a numeric vector of length %d (number of rotors configured)!',obj.numberOfRotors_)
            end
        end        
        function setInitialInput(obj, input)
        %SETINITIALINPUT  Sets initial multicopter velocity for simulation.
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
        
        %%% Get functions
        % General get
        function allProperties = getAll(obj)
        %GETALL  Returns all parameters within object.
        %
        %   GETALL() Returns a struct of all parameters inside the
        %   multicopter object. Struct is read-only and cannot change the
        %   multicopter configuration.
        %
        %   See also MASS, INERTIA, FRICTION, NUMBEROFROTORS, ROTOR
        
            allProperties.mass = obj.mass_;
            allProperties.inertiaTensor = obj.inertiaTensor_;
            allProperties.translationalFriction = obj.translationalFriction_;
            allProperties.numberOfRotors = obj.numberOfRotors_;
            allProperties.rotor = obj.rotor_;
            allProperties.timeStep = obj.timeStep_;
            allProperties.initialState = obj.initialState_;
            allProperties.previousState = obj.previousState_;
            allProperties.log = obj.log_;
            allProperties.isLogging = obj.isLogging_;
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
        %   I = inertia() Returns the multicopter inertia tensor. I is a
        %   symmetric 3x3 matrix or an empty matrix in case inertia has not
        %   been set.
        %
        %   See also MASS, GETALL, FRICTION, NUMBEROFROTORS, ROTOR
        
            inertiaValue = obj.inertiaTensor_;
        end        
        function frictionValue = friction(obj)
        %FRICTION  Returns the multicopter translational drag matrix.
        %   
        %   A = friction() Returns the multicopter translational body
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
        
            rotorStruct = [obj.rotor_(rotorID)];
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
        function rotorLiftCoeffValue = rotorLiftCoeff(obj, rotorID)
        %ROTORLIFTCOEFF Returns the lift coefficient of the specified rotor.
        %
        %   L = ROTORLIFTCOEFF(rotorID) Returns the lift coefficient of the
        %   specified rotor.
        %   rotorID specifies which rotor to return the lift coefficient of.
        %   rotorID can be a single value or an array of IDs.
        %   L is an array of size 1xN, where N = length(rotorID) and each
        %   vector element represents the lift coefficient associated with 
        %   the rotor specified by the respective rotorID. 
        %
        %   See also ROTOR,ROTORPOSITION, ROTORORIENTATION, ROTORINERTIA, 
        %   ROTORDRAGCOEFF, ROTORSTATUS, ROTOREFFICIENCY, ROTORMAXSPEED.
        
            rotorLiftCoeffValue = [obj.rotor_(rotorID).liftCoeff];
        end        
        function rotorDragCoeffValue = rotorDragCoeff(obj, rotorID)
        %ROTORDRAGCOEFF Returns the drag coefficient of the specified rotor.
        %
        %   D = ROTORDRAGCOEFF(rotorID) Returns the drag coefficient of the
        %   specified rotor.
        %   rotorID specifies which rotor to return the drag coefficient of.
        %   rotorID can be a single value or an array of IDs.
        %   D is an array of size 1xN, where N = length(rotorID) and each
        %   vector element represents the drag coefficient associated with 
        %   the rotor specified by the respective rotorID. 
        %
        %   See also ROTOR,ROTORPOSITION, ROTORORIENTATION, ROTORINERTIA, 
        %   ROTORLIFTCOEFF, ROTORSTATUS, ROTOREFFICIENCY, ROTORMAXSPEED.
        
            rotorDragCoeffValue = [obj.rotor_(rotorID).dragCoeff];
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
        %   Rotor status can be 'running', 'stuck', 'non-responding',
        %   'efficiency loss', and others according to setRotorFail().
        %
        %   See also SETROTORFAIL, SETROTORRUNNING, ROTOR, ROTORPOSITION, 
        %   ROTORORIENTATION, ROTORINERTIA, ROTORLIFTCOEFF, ROTORDRAGCOEFF,
        %   ROTOREFFICIENCY, ROTORMAXSPEED.
        
            rotorStatusValue = {obj.rotor_(rotorID).status};
        end       
        function rotorEfficiencyValue = rotorEfficiency(obj, rotorID)
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
        %   Rotor efficiency can be altered by calling setRotorFail().
        %
        %   See also SETROTORFAIL, SETROTORRUNNING, ROTOR, ROTORPOSITION, 
        %   ROTORORIENTATION, ROTORINERTIA, ROTORLIFTCOEFF, ROTORDRAGCOEFF,
        %   ROTORSTATUS, ROTORMAXSPEED.
        
            rotorEfficiencyValue = [obj.rotor_(rotorID).efficiency];
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
        %   multicopter's initial states, rotor speeds and inputs. i is 
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
                initialStateVectorValue = [initialStateVectorValue; obj.initialState_.rotor(:).speed'];
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
        function initialRotorSpeedVector = initialRotorSpeed(obj, rotorID)
        %INITIALROTORSPEED Returns the initial rotor speeds.
        %
        %   V = INITIALROTORSPEED(rotorID) Returns the initial rotor speeds 
        %   of the specified rotors.   
        %   rotorID specifies which rotor to return the efficiency of.
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
        %   multicopter's previous states, rotor speeds and inputs. i is 
        %   a numeric vector of length 20+2*nR, where nR is the number of 
        %   rotors already added to this multicopter.
        %   
        %   i is comprised of position, attitude, velocity, angular
        %   velocity, acceleration, angular acceleration, rotor speeds,
        %   inputs and previous time, in this order.
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
        %   rotorID specifies which rotor to return the efficiency of.
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
        
        function logMatrix = log(obj)
        %LOG Returns the history of all simulation steps.
        %
        %   H = LOG() Returns all previous simulation results and inputs
        %   for when logging was habilitated. 
        %   H is a matrix where each row represent a simulation step and
        %   each column a time, state or input value.
        %   H is comprised of [time, position, attitude, velocity, angular
        %   velocity, inputs], in this order. atitude is in quaternion
        %   form.
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
        
            logMatrix = obj.log_;
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
            obj.inertiaTensor_ = [];
            obj.translationalFriction_ = [];
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
        %   See also RESET, SETROTORRUNNING, SETROTORFAIL, CLEARLOG
        
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
                        else
                            error('Cannot start simulation. Configure initial states')
                        end
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
                    y0 = [obj.previousState_.position; obj.previousState_.attitude; obj.previousState_.velocity; obj.previousState_.angularVelocity];
                    opts = odeset('RelTol',1e-2,'AbsTol',1e-4);
                    [t,output] = ode45(@(t,y) obj.model(t,y,simTime,simInput), simTime, y0, opts);
                    obj.previousState_.position = output(end,1:3)';
                    obj.previousState_.attitude = output(end,4:7)';
                    obj.previousState_.velocity = output(end,8:10)';
                    obj.previousState_.angularVelocity = output(end,11:13)';
                    obj.previousState_.time = t(end);
                    obj.previousState_.input = simInput(:,end);
                    y0 = [obj.previousState_.position; obj.previousState_.attitude; obj.previousState_.velocity; obj.previousState_.angularVelocity];
                    dydt = obj.model(obj.previousState_.time,y0,simTime,simInput);
                    obj.previousState_.acceleration = dydt(8:10);
                    obj.previousState_.angularAcceleration = dydt(11:13);
                    t(1) = [];
                    output(1,:) = [];  
                    if obj.isLogging_ == true
                        if size(simInput,2)==2
                            simInput = interp1(simTime, simInput',t);
                            aux = [t,output,simInput];
                        else
                            aux = [t,output,simInput(:,2:end)'];
                        end
                        sizeaux = size(aux,2);
                        sizelog = size(obj.log_,2);
                        if sizeaux >= sizelog
                            obj.log_ = [obj.log_, zeros(1,sizeaux-sizelog);aux];
                        else
                            obj.log_ = [obj.log_; aux, zeros(1,sizelog-sizeaux)];
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
        %   SETROTORFAIL
        
            obj.isRunning_ = false;
        end        
        function setRotorRunning(obj, rotorID) 
        %SETROTORRUNNING Sets the rotor as running (normal functioning).
        %
        %   SETROTORRUNNING(rotorID) Allows rotor to run normally and sets
        %   rotor efficiency as 100%. rotorID may be scalar or an arbitrary
        %   vector of IDs.
        %   
        %   See also RUN, RESET, STOPLOGGING, CLEARLOG, SETROTORFAIL
    
            aux = cellstr('running');
            [obj.rotor_(rotorID).status] = deal(aux{1});
            [obj.rotor_(rotorID).efficiency] = deal(1);
        end        
        function setRotorFail(obj, rotorID, failType, varargin)
        %SETROTORFAIL Simulates rotor fail.
        %
        %   SETROTORFAIL(rotorID) Does nothing.
        %
        %   SETROTORFAIL(rotorID, failType) Specifies rotor fail as
        %   failType. 
        %   rotorID may be scalar or an arbitrary vector of rotor IDs. 
        %   failType must be a string specifying the type of fail. Fail
        %   types are:
        %       - 'stuck': rotor speeds are set to zero immediately. NOT
        %       IMPLEMENTED.
        %       - 'non-responding': rotor speed set points are frozen to
        %       the last rotor speeds. NOT IMPLEMENTED.
        %       - 'efficiency loss': NOT IMPLEMENTED.
        %   
        %   See also RUN, RESET, STOPLOGGING, CLEARLOG, SETROTORRUNNING
     
            switch failType
                case 'stuck'
                    aux = cellstr('stuck');
                    [obj.rotor_(rotorID).status] = deal(aux{1});
                case 'non-responding'
                    aux = cellstr('non-responding');
                    [obj.rotor_(rotorID).status] = deal(aux{1});
                    aux = mat2cell([obj.previousState_.rotor(rotorID).speed],1,ones(1,length(rotorID)));
                    [obj.rotor_(rotorID).speedSetPoint] = aux{:};
                case 'efficiency loss'
                    aux = cellstr('efficiency loss');
                    [obj.rotor_(rotorID).status] = deal(aux{1});
                    efficiency = num2cell(varargin{1});
                    [obj.rotor_(rotorID).efficiency] = efficiency{:};
                otherwise
                    error('No known error type specified');
            end
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
    
    methods (Access = private)
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

