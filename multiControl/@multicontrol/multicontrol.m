classdef multicontrol < multicopter
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties %(Access = private)
        % General
        trajectoryMap_      %Map that contains the polynomial trajectory matrices for each axis and period of time
        trajectory_         %Struct that contains the desired trajectory in space and attitude reference for each time step
        rotorDirection_     %Array containing the rotor directions of rotation around the orientation axis. Follows the right-hand rule
        rotorOperatingPoint_% Array containing the hover operating point velocities for each rotor.
        metrics_            %Struct containing the metrics to evaluate the controllers performances
        
        % Simulation settings
        controlAlg_         %Control algorithm used in simulation
        allocationAlg_      %Control allocation algorithm used in simulation        
        attReferenceAlg_    %Control allocation algorithm used in attitude reference method
        
        controlTimeStep_    %Time step for the multirotor control. Should not be mistaken with the multirotor simulator time step
        controlDelay_       %Percentage of control time step that the algorithm takes to calculate and apply inputs to multirotor
        
        % Algorithms configurations
        controlConfig_          %Cell that contains all possible attitude control architectures configurations
        positionControlConfig_  %Struct that contains the PIDD position gains
        allocationConfig_       %Cell that contains all possible control allocation architectures configurations
        fddConfig_              %Struct with FDD rate and delay
        automationConfig_       %Cell that contains the types and intensities of faults and uncertainties at specified periods 
        filterConfig_           %Low pass filter for the desired velocities, to prevent large noises
        
        % Auxiliary variables
        timeStepRelation_       %Controller time step must be timeStepRelation_ times greater than simulation time step
        attitudeControllers_    %List of attitude controllers available for simulation
        controlAllocators_      %List of control allocation methods available for simulation
        fddRotor_               %Struct for each motor containing motor status and efficiency for each time step until FDD maximum delay
        velocityFilter_         %Struct containing variables necessary to filter velocities and define desired velocites and accelerations
        roulette
        debug
    end
    
    methods
        %%% Constructor
        function obj = multicontrol(N)
        %MULTICONTROL Creates a multicontrol object for simulation of a multicopter.
        %
        %   MMULTICONTROL() Creates an empty multicopter object with initial
        %   position and velocities set to zero. Call set functions to 
        %   configure multicopter for simulation.
        %
        %   MULTICONTROL(N) Creates a default multicopter with N rotors
        %   equally distributed and ready for simulation, based on
        %   parameters from "Modelling and control of quadcopter",
        %   Luukonen, 2011. Distributes rotors equally over xy plane in X
        %   configuration. The first rotor is always the first to the
        %   CW direction after the x axis in the xy plane. All other
        %   rotors are added in CCW direction starting from rotor 1.
        %
        %   See also MULTICOPTER, SETMASS, SETINERTIA, SETFRICTION, ADDROTOR, SETROTOR,
        %   SETTIMESTEP, SETINITIALSTATE, STARTLOGGING, RUN, RESET.
                       
            % Creates multicopter
            obj = obj@multicopter(N);
            obj.attitudeControllers_ = {'PID',...
                                        'RLQ-R Passive',...
                                        'RLQ-R Passive Modified',...
                                        'RLQ-R Passive Modified with PIDD',...
                                        'RLQ-R Active',...
                                        'RLQ-R Active Modified',...
                                        'RLQ-R Active Modified with PIDD',...
                                        'SOSMC Passive',...
                                        'SOSMC Passive with PIDD',...
                                        'SOSMC Passive Direct',...
                                        'SOSMC Active',...
                                        'SOSMC Active with PIDD',...
                                        'SOSMC Active Direct',...
                                        'Adaptive',...
                                        'Adaptive with PIDD',...
                                        'Adaptive Direct',...
                                        'Markovian RLQ-R Passive Modified',...
                                        'Markovian RLQ-R Active Modified'};
                                    
            obj.controlAllocators_ =   {'PI Passive',...
                                        'PI Active',...
                                        'RPI Passive',...
                                        'RPI Active',...
                                        'Adaptive',...
                                        'CLS Passive',...
                                        'Passive NMAC',...
                                        'Active NMAC',...
                                        'None'};            
        
            % Sets variables sizes and empties
            obj.trajectoryMap_.endTime  = [];
            obj.trajectoryMap_.type     = 'waypoints';
            obj.trajectoryMap_.ax       = {};
            obj.trajectoryMap_.ay       = {};
            obj.trajectoryMap_.az       = {};
            obj.trajectoryMap_.ayaw     = {};
            obj.clearTrajectory();
            obj.rotorDirection_         = zeros(1,obj.numberOfRotors_); 
            obj.rotorOperatingPoint_    = zeros(1,obj.numberOfRotors_);
            obj.clearMetrics();
            
            obj.controlAlg_         = [];
            obj.allocationAlg_      = [];
            obj.attReferenceAlg_    = [];
            
            obj.controlTimeStep_    = 0.1;
            obj.timeStep_           = obj.controlTimeStep_/10.0;
            obj.controlDelay_       = 0.1;
            
            obj.controlConfig_      = {};
            obj.allocationConfig_   = {};
            obj.fddConfig_.hitRate  = [];
            obj.fddConfig_.delay    = [];
            
            obj.automationConfig_.time      = [];
            obj.automationConfig_.commands  = {};    
            obj.filterConfig_.angular       = [0;0;0.5];
            obj.filterConfig_.linear        = [];
            
            obj.timeStepRelation_   = 10;
            obj.clearFDD();
            
            obj.positionControlConfig_.kp = [];
            obj.positionControlConfig_.kd = [];
            obj.positionControlConfig_.kdd = [];
        end
        
        %%% Public Methods
        % blabla
        % Set and config methods
        function setController(obj, type)
        %UNTITLED Summary of this function goes here
        %   Detailed explanation goes here
            if ~iscell(type) && ischar(type)
                if any(strcmp(type,obj.attitudeControllers_))
                    obj.controlAlg_ = find(strcmp(type,obj.attitudeControllers_));
                    if obj.verbose_ == true
                        disp(['Controller method set to "',type,'".']);
                    end
                else
                    error('There is no controller "%s" available',type);
                end
            else
                error('Controller type must be a single string containing a valid controller');
            end
        end  
        function configController(obj, varargin)
        %UNTITLED Summary of this function goes here
        %   Detailed explanation goes here
            if length(varargin)>=1 && ~isempty(varargin{1})
                it = 1;
                while it<=length(varargin)
                    switch varargin{it}
                        case 'Position PIDD' % Position controller
                            if obj.inputsOK(varargin,it,4)
                                it = it+1;
                                if isnumeric(varargin{it}) && (isequal(size(varargin{it}),[1 3]) || isequal(size(varargin{it}),[3 1]))
                                    if isnumeric(varargin{it+1}) && (isequal(size(varargin{it+1}),[1 3]) || isequal(size(varargin{it+1}),[3 1]))
                                        if isnumeric(varargin{it+2}) && (isequal(size(varargin{it+2}),[1 3]) || isequal(size(varargin{it+2}),[3 1]))
                                            if isnumeric(varargin{it+3}) && (isequal(size(varargin{it+3}),[1 3]) || isequal(size(varargin{it+3}),[3 1]))
                                                obj.positionControlConfig_.kp = varargin{it};
                                                obj.positionControlConfig_.ki = varargin{it+1};
                                                obj.positionControlConfig_.kd = varargin{it+2};
                                                obj.positionControlConfig_.kdd = varargin{it+3};
                                                
                                                if obj.verbose_ == true
                                                    disp(['Position controller proportional gain set to [',num2str(reshape(obj.positionControlConfig_.kp,[1 3])),'].']);
                                                    disp(['Position controller integral gain set to [',num2str(reshape(obj.positionControlConfig_.ki,[1 3])),'].']);
                                                    disp(['Position controller derivative gain set to [',num2str(reshape(obj.positionControlConfig_.kd,[1 3])),'].']);
                                                    disp(['Position controller second derivative gain set to [',num2str(reshape(obj.positionControlConfig_.kdd,[1 3])),'].']);
                                                end
                                            else
                                                warning('Position controller second derivative gain must be a numeric array of size 1x3 or 3x1. Skipping this controller configuration');
                                            end    
                                        else
                                            warning('Position controller derivative gain must be a numeric array of size 1x3 or 3x1. Skipping this controller configuration');
                                        end
                                    else
                                        warning('Position controller integral gain must be a numeric array of size 1x3 or 3x1. Skipping this controller configuration');
                                    end                                    
                                else
                                    warning('Position controller proportional gain must be a numeric array of size 1x3 or 3x1. Skipping this controller configuration');
                                end
                                it = it+3;
                            else
                                warning('Missing argument for position controller. Skipping configuration.');
                            end
                        case obj.attitudeControllers_{1} %% PID
                            if obj.inputsOK(varargin,it,3)
                                it = it+1;
                                if isnumeric(varargin{it}) && (isequal(size(varargin{it}),[1 3]) || isequal(size(varargin{it}),[3 1]))
                                    if isnumeric(varargin{it+1}) && (isequal(size(varargin{it+1}),[1 3]) || isequal(size(varargin{it+1}),[3 1]))
                                        if isnumeric(varargin{it+2}) && (isequal(size(varargin{it+2}),[1 3]) || isequal(size(varargin{it+2}),[3 1]))
                                            obj.controlConfig_{1}.kp = varargin{it};
                                            obj.controlConfig_{1}.ki = varargin{it+1};
                                            obj.controlConfig_{1}.kd = varargin{it+2};
                                            
                                            if obj.verbose_ == true
                                                disp([obj.attitudeControllers_{1},' Attitude controller proportional gain set to [',num2str(reshape(obj.controlConfig_{1}.kp,[1 3])),'].']);
                                                disp([obj.attitudeControllers_{1},' Attitude controller integral gain set to [',num2str(reshape(obj.controlConfig_{1}.ki,[1 3])),'].']);
                                                disp([obj.attitudeControllers_{1},' Attitude controller derivative gain set to [',num2str(reshape(obj.controlConfig_{1}.kd,[1 3])),'].']);
                                            end
                                        else
                                            warning('Attitude controller derivative gain must be a numeric array of size 1x3 or 3x1. Skipping this controller configuration');
                                        end   
                                    else
                                        warning('Attitude controller integral gain must be a numeric array of size 1x3 or 3x1. Skipping this controller configuration');
                                    end  
                                else
                                    warning('Attitude controller proportional gain must be a numeric array of size 1x3 or 3x1. Skipping this controller configuration');
                                end
                                it = it+2;                                
                            else
                                warning('Missing argument for attitude %s controller. Skipping configuration.',obj.attitudeControllers_{1});
                            end
                        case obj.attitudeControllers_{2} %% RLQ-R Passive
                            it = obj.configureRLQ(2,varargin,it);
                        case obj.attitudeControllers_{3} %% RLQ-R Passive Modified
                            it = obj.configureRLQ(3,varargin,it);
                        case obj.attitudeControllers_{4} %% RLQ-R Passive Modified with PIDD
                            it = obj.configureRLQ(4,varargin,it);
                        case obj.attitudeControllers_{5} %% RLQ-R Active
                            it = obj.configureRLQ(5,varargin,it);
                        case obj.attitudeControllers_{6} %% RLQ-R Active Modified
                            it = obj.configureRLQ(6,varargin,it);
                        case obj.attitudeControllers_{7} %% RLQ-R Active Modified with PIDD
                            it = obj.configureRLQ(7,varargin,it);
                        case obj.attitudeControllers_{8} %% SOSMC Passive
                            it = obj.configureSOSMC(8,varargin,it);
                        case obj.attitudeControllers_{9} %% SOSMC Passive with PIDD
                            it = obj.configureSOSMC(9,varargin,it);
                        case obj.attitudeControllers_{10} %% SOSMC Passive Direct
                            index = 10;
                            if obj.inputsOK(varargin,it,5)
                                it = it+1;
                                if isnumeric(varargin{it}) && isdiag(varargin{it}) && all(diag(varargin{it})>=0)
                                    if isnumeric(varargin{it+1}) %& all(varargin{it+1}>=0)
                                        if isnumeric(varargin{it+2}) %& all(varargin{it+2}>=0)
                                            if isnumeric(varargin{it+3}) && isscalar(varargin{it+3})
                                                if isnumeric(varargin{it+4}) && isscalar(varargin{it+4})
                                                    obj.controlConfig_{index}.c = varargin{it};
                                                    obj.controlConfig_{index}.lambda = varargin{it+1};
                                                    obj.controlConfig_{index}.alpha = varargin{it+2};
                                                    obj.allocationConfig_{index}.R = varargin{it+3};
                                                    obj.allocationConfig_{index}.Q = varargin{it+4};

                                                    if obj.verbose_ == true
                                                        disp([obj.attitudeControllers_{index},' Attitude controller c error gain set to: ']);
                                                        disp(obj.controlConfig_{index}.c);
                                                        disp([obj.attitudeControllers_{index},' Attitude controller Lambda gain set to: '])
                                                        disp(obj.controlConfig_{index}.lambda);
                                                        disp([obj.attitudeControllers_{index},' Attitude controller Alpha gain set to: '])
                                                        disp(obj.controlConfig_{index}.alpha);
                                                        disp(['Maneuverability factor set to: ',num2str(obj.allocationConfig_{index}.R)])
                                                        disp(['Attitude factor set to: ',num2str(obj.allocationConfig_{index}.Q)])
                                                    end
                                                else
                                                    warning('Attitude factor must be numeric and scalar. Skipping this controller configuration');
                                                end
                                            else
                                                warning('Maneuverability factor must be numeric and scalar. Skipping this controller configuration');
                                            end
                                        else
                                            warning('Attitude controller Alpha gain must be a numeric matrix. Skipping this controller configuration');
                                        end
                                    else
                                        warning('Attitude controller Lambda gain must be a numeric matrix. Skipping this controller configuration');
                                    end
                                else
                                    warning('Attitude controller c gain must be a positive numeric diagonal matrix. Skipping this controller configuration');
                                end
                                it = it+4;
                            else
                                warning('Missing argument for attitude %s controller. Skipping this controller configuration.',obj.attitudeControllers_{index});
                            end
                        case obj.attitudeControllers_{11} %% SOSMC Active
                            it = obj.configureSOSMC(11,varargin,it);
                        case obj.attitudeControllers_{12} %% SOSMC Active with PIDD
                            it = obj.configureSOSMC(12,varargin,it);
                        case obj.attitudeControllers_{13} %% SOSMC Active Direct
                            index = 13;
                            if obj.inputsOK(varargin,it,5)
                                it = it+1;
                                if isnumeric(varargin{it}) && isdiag(varargin{it}) && all(diag(varargin{it})>=0)
                                    if isnumeric(varargin{it+1}) %& all(varargin{it+1}>=0)
                                        if isnumeric(varargin{it+2}) %& all(varargin{it+2}>=0)
                                            if isnumeric(varargin{it+3}) && isscalar(varargin{it+3})
                                                if isnumeric(varargin{it+4}) && isscalar(varargin{it+4})
                                                    obj.controlConfig_{index}.c = varargin{it};
                                                    obj.controlConfig_{index}.lambda = varargin{it+1};
                                                    obj.controlConfig_{index}.alpha = varargin{it+2};
                                                    obj.allocationConfig_{index}.R = varargin{it+3};
                                                    obj.allocationConfig_{index}.Q = varargin{it+4};

                                                    if obj.verbose_ == true
                                                        disp([obj.attitudeControllers_{index},' Attitude controller c error gain set to: ']);
                                                        disp(obj.controlConfig_{index}.c);
                                                        disp([obj.attitudeControllers_{index},' Attitude controller Lambda gain set to: '])
                                                        disp(obj.controlConfig_{index}.lambda);
                                                        disp([obj.attitudeControllers_{index},' Attitude controller Alpha gain set to: '])
                                                        disp(obj.controlConfig_{index}.alpha);
                                                        disp(['Maneuverability factor set to: ',num2str(obj.allocationConfig_{index}.R)])
                                                        disp(['Attitude factor set to: ',num2str(obj.allocationConfig_{index}.Q)])
                                                    end
                                                else
                                                    warning('Attitude factor must be numeric and scalar. Skipping this controller configuration');
                                                end
                                            else
                                                warning('Maneuverability factor must be numeric and scalar. Skipping this controller configuration');
                                            end
                                        else
                                            warning('Attitude controller Alpha gain must be a numeric matrix. Skipping this controller configuration');
                                        end
                                    else
                                        warning('Attitude controller Lambda gain must be a numeric matrix. Skipping this controller configuration');
                                    end
                                else
                                    warning('Attitude controller c gain must be a positive numeric diagonal matrix. Skipping this controller configuration');
                                end
                                it = it+4;
                            else
                                warning('Missing argument for attitude %s controller. Skipping this controller configuration.',obj.attitudeControllers_{index});
                            end
                        case obj.attitudeControllers_{14} %% Adaptive
                            it = obj.configureAdaptive(14,varargin,it);
                        case obj.attitudeControllers_{15} %% Adaptive with PIDD
                            it = obj.configureAdaptive(15,varargin,it);
                        case obj.attitudeControllers_{16} %% Adaptive Direct
                            index = 16;
                            if obj.inputsOK(varargin,it,7)
                                it = it+1;
                                if isnumeric(varargin{it}) && all(real(eig(varargin{it}))<=0)
                                    if isnumeric(varargin{it+1})
                                        if isnumeric(varargin{it+2})
                                            if isnumeric(varargin{it+3})
                                                if isnumeric(varargin{it+4})
                                                    if isnumeric(varargin{it+5})
                                                        if isnumeric(varargin{it+6}) && isequal(size(varargin{it+6}),[obj.numberOfRotors(),6])
                                                            obj.controlConfig_{index}.Am = varargin{it};
                                                            obj.controlConfig_{index}.Q = varargin{it+1};                                     
                                                            obj.controlConfig_{index}.gamma1 = varargin{it+2};
                                                            obj.controlConfig_{index}.gamma2 = varargin{it+3};
                                                            obj.controlConfig_{index}.gamma3 = varargin{it+4};
                                                            obj.controlConfig_{index}.gamma4 = varargin{it+5};
                                                            obj.controlConfig_{index}.B0 = varargin{it+6};

                                                            if obj.verbose_ == true
                                                                disp([obj.attitudeControllers_{index},' Adaptive reference matrix Am set to: ']);
                                                                disp(obj.controlConfig_{index}.Am);
                                                                disp([obj.attitudeControllers_{index},' Adaptive matrix Q set to: '])
                                                                disp(obj.controlConfig_{index}.Q);   
                                                                disp([obj.attitudeControllers_{index},' Adaptive gain Gamma 1 set to: '])
                                                                disp(obj.controlConfig_{index}.gamma1);
                                                                disp([obj.attitudeControllers_{index},' Adaptive gain Gamma 2 set to: '])
                                                                disp(obj.controlConfig_{index}.gamma2);
                                                                disp([obj.attitudeControllers_{index},' Adaptive gain Gamma 3 set to: '])
                                                                disp(obj.controlConfig_{index}.gamma3);
                                                                disp([obj.attitudeControllers_{index},' Adaptive gain Gamma 4 set to: '])
                                                                disp(obj.controlConfig_{index}.gamma4);
                                                                disp([obj.attitudeControllers_{index},' Adaptive initial input matrix estimate set to: '])
                                                                disp(obj.controlConfig_{index}.B0);
                                                            end
                                                        else
                                                            warning('Initial input matrix must be a numeric matrix. Skipping this controller configuration');
                                                        end
                                                    else
                                                        warning('Gain Gamma 4 must be a numeric matrix. Skipping this controller configuration');
                                                    end
                                                else
                                                    warning('Gain Gamma 3 must be a numeric matrix. Skipping this controller configuration');
                                                end
                                            else
                                                warning('Gain Gamma 2 must be a numeric matrix. Skipping this controller configuration');
                                            end
                                        else
                                            warning('Gain Gamma 1 must be a numeric matrix. Skipping this controller configuration');
                                        end
                                    else
                                        warning('Q must be a numeric matrix. Skipping this controller configuration');
                                    end
                                else
                                    warning('Reference matrix Am must be a numeric stable matrix. Skipping this controller configuration');
                                end
                                it = it+6;
                            else
                                warning('Missing argument for attitude %s controller. Skipping this controller configuration.',obj.attitudeControllers_{index});
                            end
                        case obj.attitudeControllers_{17} %% Markovian RLQ-R Passive Modified
                            index = 17;
                            if obj.inputsOK(varargin,it,10)
                                it = it+1;
                                if isnumeric(varargin{it}) && sum(size(varargin{it}))>2 && size(varargin{it},2)==obj.numberOfRotors_
                                    numberOfModes = size(varargin{it},1);
                                    if isnumeric(varargin{it+1}) && sum(size(varargin{it+1}))>2
                                        if isnumeric(varargin{it+2}) && size(varargin{it+2},2)==6 && size(varargin{it+2},3)==numberOfModes
                                            if isnumeric(varargin{it+3}) && size(varargin{it+3},2)==obj.numberOfRotors_ && size(varargin{it+3},3)==numberOfModes
                                                if isnumeric(varargin{it+4}) && sum(size(varargin{it+4}))<=2 && varargin{it+4}>=1
                                                    if isnumeric(varargin{it+5}) && size(varargin{it+5},1)==size(varargin{it+5},2) && size(varargin{it+5},1)==obj.numberOfRotors_ && size(varargin{it+5},3)==numberOfModes
                                                        if isnumeric(varargin{it+6}) && size(varargin{it+6},1)==size(varargin{it+6},2) && size(varargin{it+6},1)==6 && size(varargin{it+6},3)==numberOfModes
                                                            if isnumeric(varargin{it+7}) && sum(size(varargin{it+7}))<=2 && varargin{it+7}>0
                                                                if isnumeric(varargin{it+8}) && isequal(size(varargin{it+8}),[numberOfModes numberOfModes])
                                                                    if isnumeric(varargin{it+9}) && isequal(size(varargin{it+9}),[numberOfModes numberOfModes])

                                                                        obj.controlConfig_{index}.modes = varargin{it};
                                                                        obj.controlConfig_{index}.initialP = varargin{it+1};
                                                                        obj.controlConfig_{index}.Ef = varargin{it+2};
                                                                        obj.controlConfig_{index}.Eg = varargin{it+3};
                                                                        obj.controlConfig_{index}.k = varargin{it+4};
                                                                        obj.controlConfig_{index}.Er = varargin{it+5};
                                                                        obj.controlConfig_{index}.Eq = varargin{it+6};
                                                                        obj.controlConfig_{index}.lambda = varargin{it+7};
                                                                        obj.controlConfig_{index}.pij = varargin{it+8};
                                                                        obj.controlConfig_{index}.eij = varargin{it+9};

                                                                        if obj.verbose_ == true
                                                                            disp([obj.attitudeControllers_{index},' Attitude controller markovian modes number set to: '])
                                                                            disp(size(obj.controlConfig_{index}.modes,1));
                                                                            disp('Markovian Passive Modified Attitude controller set.')
                                                                        end
                                                                    else
                                                                        warning('Attitude controller probability matrix error should be a numeric square matrix of the same size as the number of markovian modes. Skipping this controller configuration');
                                                                    end     
                                                                else
                                                                    warning('Attitude controller probability matrix nominal values should be a numeric square matrix of the same size as the number of markovian modes. Skipping this controller configuration');
                                                                end            
                                                            else
                                                                warning('Attitude controller lambda must be a scalar > 0. Skipping this controller configuration.');
                                                            end
                                                        else
                                                            warning('Attitude controller Eq must be a sqaure matrix of size 6x6. Skipping this controller configuration.');
                                                        end
                                                    else
                                                        warning('Attitude controller Er must be a matrix of size N x N, where N is the number of rotors. Skipping this controller configuration.');
                                                    end        
                                                else
                                                    warning('Attitude controller scalar k must be a scalar >= 1. Skipping this controller configuration.');
                                                end
                                            else
                                                warning('Attitude controller uncertainty Eg must be a numeric matrix. Skipping this controller configuration.');
                                            end
                                        else
                                            warning('Attitude controller uncertainty Ef must be a numeric matrix. Skipping this controller configuration.');
                                        end
                                    else
                                        warning('Attitude controller initial state variance must be a numeric matrix. Skipping this controller configuration.');
                                    end
                                else
                                    warning('Attitude controller markovian modes be a numeric matrix. The number of rows correspond to the number of markovian modes. The number of columns correspond to the number of rotors. A value of 0 means the rotor is faulty while a value of 1 means the rotor is fully operational. Skipping this controller configuration.');
                                end
                                it = it+11;                
                            else
                                warning('Missing argument for attitude %s controller. Skipping this controller configuration.',obj.attitudeControllers_{index});
                            end
                        case obj.attitudeControllers_{18} %% Markovian RLQ-R Active Modified
                            index = 18;
                            if obj.inputsOK(varargin,it,12)
                                it = it+1;
                                if isnumeric(varargin{it}) && sum(size(varargin{it}))>2 && size(varargin{it},2)==obj.numberOfRotors_
                                numberOfModes = size(varargin{it},1);
                                    if isnumeric(varargin{it+1}) && sum(size(varargin{it+1}))>2 && size(varargin{it+1},3)==numberOfModes
                                        if isnumeric(varargin{it+2}) && size(varargin{it+2},1)==size(varargin{it+2},2) && size(varargin{it+2},1)==6 && size(varargin{it+2},3)==numberOfModes
                                            if isnumeric(varargin{it+3}) && size(varargin{it+3},1)==size(varargin{it+3},2) && size(varargin{it+3},1)==obj.numberOfRotors_ && size(varargin{it+3},3)==numberOfModes
                                                if isnumeric(varargin{it+4}) && size(varargin{it+4},2)==6 && size(varargin{it+4},3)==numberOfModes
                                                    if isnumeric(varargin{it+5}) && size(varargin{it+5},2)==obj.numberOfRotors_ && size(varargin{it+5},3)==numberOfModes
                                                        if isnumeric(varargin{it+6}) && size(varargin{it+6},1)==6 && size(varargin{it+6},3)==numberOfModes
                                                            if isnumeric(varargin{it+7}) && isequal(size(varargin{it+7}),[numberOfModes numberOfModes])
                                                                if isnumeric(varargin{it+8}) && length(varargin{it+8})==numberOfModes
                                                                    if isnumeric(varargin{it+9}) && sum(size(varargin{it+9}))==2 && varargin{it+9}>=1
                                                                        if isnumeric(varargin{it+10}) && sum(size(varargin{it+10}))==2 && varargin{it+10}>0
                                                                            if isnumeric(varargin{it+11}) && sum(size(varargin{it+11}))==2 && varargin{it+11}>=0

                                                                                obj.controlConfig_{index}.modes = varargin{it};
                                                                                obj.controlConfig_{index}.initialP = varargin{it+1};
                                                                                obj.controlConfig_{index}.Q = varargin{it+2};
                                                                                obj.controlConfig_{index}.R = varargin{it+3};
                                                                                obj.controlConfig_{index}.Ef = varargin{it+4};
                                                                                obj.controlConfig_{index}.Eg = varargin{it+5};
                                                                                obj.controlConfig_{index}.H = varargin{it+6};
                                                                                obj.controlConfig_{index}.pij = varargin{it+7};
                                                                                obj.controlConfig_{index}.ei = varargin{it+8};
                                                                                obj.controlConfig_{index}.k = varargin{it+9};
                                                                                obj.controlConfig_{index}.mu = varargin{it+10};
                                                                                obj.controlConfig_{index}.alpha = varargin{it+11};

                                                                            if obj.verbose_ == true
                                                                                disp([obj.attitudeControllers_{index},' Attitude controller markovian modes number set to: '])
                                                                                disp(size(obj.controlConfig_{index}.modes,1));
                                                                                disp(' Markovian active modified attitude controller set.');
                                                                            end
                                                                            else
                                                                                warning('Attitude controller alpha should be a scalar > 0. Skipping this controller configuration');
                                                                            end
                                                                        else
                                                                            warning('Attitude controller mu should be a scalar > 0. Skipping this controller configuration');
                                                                        end
                                                                    else
                                                                        warning('Attitude controller scalar k must be a scalar >= 1. Skipping this controller configuration');
                                                                    end     
                                                                else
                                                                    warning('Attitude controller probability matrix error should be a numeric vector of length numberOfModes. Skipping this controller configuration.');
                                                                end            
                                                            else
                                                                warning('Attitude controller probability matrix nominal values should be a numeric square matrix of the same size as the number of markovian modes. Skipping this controller configuration');
                                                            end
                                                        else
                                                            warning('Attitude controller uncertainty H must be a numeric matrix of size 6 x t. Skipping this controller configuration');
                                                        end
                                                    else
                                                        warning('Attitude controller uncertainty Eg must be a numeric matrix of size l x numberOrRotors x numberOfModes. Skipping this controller configuration.');
                                                    end        
                                                else
                                                    warning('Attitude controller uncertainty Ef must be a numeric matrix of size l x 6 x numberOfModes. Skipping this controller configuration');
                                                end
                                            else
                                                warning('Attitude controller R matrix should be of size numberOfRotors x numberOfRotors x numberOfModes. Skipping this controller configuration.');
                                            end
                                        else
                                            warning('Attitude controller Q matrix should be of size 6 x 6 x numberOfModes. Skipping this controller configuration.');
                                        end
                                    else
                                        warning('Attitude controller initial state variance must be a numeric matrix. Skipping this controller configuration.');
                                    end
                                else
                                    warning('Attitude controller markovian modes be a numeric matrix. The number of rows correspond to the number of markovian modes. The number of columns correspond to the number of rotors. A value of 0 means the rotor is faulty while a value of 1 means the rotor is fully operational. Skipping this controller configuration.');
                                end
                                it = it+13;                
                            else
                                warning('Missing argument for attitude %s controller. Skipping this controller configuration.',obj.attitudeControllers_{index});
                            end
                        otherwise
                            warning('Controller type not expected. Skipping configuration');
                    end
                    it = it+1;
                    while it<=length(varargin) && ~ischar(varargin{it})
                        it = it+1;
                    end
                end
            end
        end
        function configControlAllocator(obj, varargin)
        %UNTITLED Summary of this function goes here
        %   Detailed explanation goes here
            if length(varargin)>=1 && ~isempty(varargin{1})
                it = 1;
                while it<=length(varargin)
                    switch varargin{it}
                        case 'Adaptive' % Adaptive control allocator
                            index = 5;
                            if obj.inputsOK(varargin,it,3)
                                it = it+1;
                                if isnumeric(varargin{it})
                                    if isnumeric(varargin{it+1}) && isscalar(varargin{it+1})
                                        if isnumeric(varargin{it+2}) && isscalar(varargin{it+2})
                                            obj.allocationConfig_{index}.Am = varargin{it};
                                            obj.allocationConfig_{index}.R = varargin{it+1};
                                            obj.allocationConfig_{index}.Q = varargin{it+2};
                                            if obj.verbose_ == true
                                                disp('Adaptive control allocator gain set to: ')
                                                disp(obj.allocationConfig_{index}.Am);
                                                disp(['Adaptive control allocator maneuverability factor set to: ',num2str(obj.allocationConfig_{index}.R)])
                                                disp(['Adaptive control allocator attitude factor set to: ',num2str(obj.allocationConfig_{index}.Q)])
                                            end
                                        else            
                                            warning('Attitude factor must be numeric and scalar. Skipping this control allocation configuration');
                                        end
                                    else
                                        warning('Maneuverability factor must be numeric and scalar. Skipping this control allocation configuration');
                                    end
                                else
                                    warning('Adaptive gain must be numeric. Skipping this control allocation configuration');
                                end
                                it = it+1;
                            else
                                warning('Missing argument for adaptive allocator. Skipping configuration.');
                            end
                        case 'Passive NMAC' % Passive Null-Space-Based Maneuverability/Attitude Contest (NMAC)
                            index = 7;
                            if obj.inputsOK(varargin,it,2)
                                it = it+1;
                                if isnumeric(varargin{it}) && isscalar(varargin{it})
                                    if isnumeric(varargin{it+1}) && isscalar(varargin{it+1})
                                        obj.allocationConfig_{index}.R = varargin{it};
                                        obj.allocationConfig_{index}.Q = varargin{it+1};
                                        if obj.verbose_ == true
                                            disp(['Maneuverability factor set to: ',num2str(obj.allocationConfig_{index}.R)])
                                            disp(['Attitude factor set to: ',num2str(obj.allocationConfig_{index}.Q)])
                                        end
                                    else
                                        warning('Attitude factor must be numeric and scalar. Skipping this control allocation configuration');
                                    end
                                else
                                    warning('Maneuverability factor must be numeric and scalar. Skipping this control allocation configuration');
                                end
                                it = it+2;
                            else
                                warning('Missing argument for Passive NMAC allocator. Skipping configuration.');
                            end
                        case 'Active NMAC' % Active Null-Space-Based Maneuverability/Attitude Contest (NMAC)
                            index = 8;
                            if obj.inputsOK(varargin,it,2)
                                it = it+1;
                                if isnumeric(varargin{it}) && isscalar(varargin{it})
                                    if isnumeric(varargin{it+1}) && isscalar(varargin{it+1})
                                        obj.allocationConfig_{index}.R = varargin{it};
                                        obj.allocationConfig_{index}.Q = varargin{it+1};
                                        if obj.verbose_ == true
                                            disp(['Maneuverability factor set to: ',num2str(obj.allocationConfig_{index}.R)])
                                            disp(['Attitude factor set to: ',num2str(obj.allocationConfig_{index}.Q)])
                                        end
                                    else
                                        warning('Attitude factor must be numeric and scalar. Skipping this control allocation configuration');
                                    end
                                else
                                    warning('Maneuverability factor must be numeric and scalar. Skipping this control allocation configuration');
                                end
                                it = it+2;
                            else
                                warning('Missing argument for Active NMAC allocator. Skipping configuration.');
                            end
                        otherwise
                            warning('Control Allocation type not expected. Skipping configuration');
                    end
                    it = it+1;
                    while it<=length(varargin) && ~ischar(varargin{it})
                        it = it+1;
                    end
                end
            end
        end        
        function setControlAllocator(obj, type)
        %UNTITLED Summary of this function goes here
        %   Detailed explanation goes here
            if ~iscell(type) && ischar(type)
                if any(strcmp(type,obj.controlAllocators_))
                    obj.allocationAlg_ = find(strcmp(type,obj.controlAllocators_));
                    if obj.verbose_ == true
                        disp(['Control allocation set to "',type,'".']);
                    end
                else
                    error('There is no control allocation "%s" available',type);
                end
            else
                error('Control allocation type must be a single string containing a valid control allocation method');
            end
        end  
        function setAttitudeReferenceCA(obj, type)
        %UNTITLED Summary of this function goes here
        %   Detailed explanation goes here
            if ~iscell(type) && ischar(type)
                if any(strcmp(type,obj.controlAllocators_))
                    obj.attReferenceAlg_ = find(strcmp(type,obj.controlAllocators_));
                    if obj.verbose_ == true
                        disp(['Attitude reference control allocation set to "',type,'".']);
                    end
                else
                    error('There is no control allocation "%s" available',type);
                end
            else
                error('Control allocation type must be a single string containing a valid control allocation method');
            end
        end
        function setTrajectory(obj, type, varargin)
        %UNTITLED Summary of this function goes here
        %   Detailed explanation goes here
            switch type
                case 'gerono'
                    if length(varargin)==5 || length(varargin)==6 || length(varargin)==8
                        if isscalar(varargin{1}) && isnumeric(varargin{1}) && varargin{1}>0 && ...
                                isscalar(varargin{2}) && isnumeric(varargin{2}) && varargin{2}>0 && ...
                                isscalar(varargin{3}) && isnumeric(varargin{3}) && varargin{3}>0 && ...
                                isscalar(varargin{4}) && isnumeric(varargin{4}) && varargin{4}>0
                            switch varargin{5}
                                case 'fixed'
                                    if length(varargin)==6 && isnumeric(varargin{6}) && isscalar(varargin{6})
                                        obj.trajectoryMap_ = [];
                                        obj.trajectoryMap_.yawType = 'fixed';
                                        obj.trajectoryMap_.yawAngle = varargin{6};
                                    else
                                        error('Must specify yaw angle. Skipping trajectory configuration')
                                    end
                                case '360'
                                        obj.trajectoryMap_ = [];
                                        obj.trajectoryMap_.yawType = '360';
                                case 'goto'
                                    if length(varargin)==6 && isnumeric(varargin{6}) && isscalar(varargin{6})                                        
                                        obj.trajectoryMap_ = [];
                                        obj.trajectoryMap_.yawType = 'goto';
                                        obj.trajectoryMap_.yawAngle = varargin{6};
                                    else
                                        error('Must specify yaw angle. Skipping trajectory configuration')
                                    end
                                case 'sinusoidal'
                                    if length(varargin)==8 && isnumeric(varargin{6}) && isscalar(varargin{6}) && ...
                                            isnumeric(varargin{7}) && isscalar(varargin{7}) && ...
                                            isnumeric(varargin{8}) && isscalar(varargin{8})                                        
                                        obj.trajectoryMap_ = [];
                                        obj.trajectoryMap_.yawType = 'sinusoidal';
                                        obj.trajectoryMap_.yawMean = varargin{6};
                                        obj.trajectoryMap_.yawMax = varargin{7};
                                        obj.trajectoryMap_.yawPeriod = varargin{8};
                                    else
                                        error('Must specify yaw mean, max and period. Skipping trajectory configuration')
                                    end  
                                otherwise
                                    error('No such yawtype available')
                            end    
                            obj.trajectoryMap_.length = varargin{1};
                            obj.trajectoryMap_.width = varargin{2};
                            obj.trajectoryMap_.height = varargin{3};
                            obj.trajectoryMap_.endTime = varargin{4};
                            obj.trajectoryMap_.type = 'gerono';
                            if obj.verbose_ == true
                                disp('Lemniscate of Gerono configured')
                            end
                        else
                            error('Length, width, height and endtime must be numeric scalars greater than 0.')
                        end
                    else
                        error('Too many or too few inputs')
                    end                    
                case 'waypoints'
                    waypoints = varargin{1};
                    time = varargin{2};
                    if ~isnumeric(time) 
                        error('Time must be numeric')
                    elseif ~size(time,1)==1
                        error('Time must be a row vector')
                    elseif ~(size(time,2)==1 || size(time,2)==size(waypoints,2))
                        error('Time must be a scalar indicating total trajectory duration or an array indicating each waypoint ending in time')
                    elseif ~issorted(time)
                        error('Time must be a crescent array')
                    elseif (time(1)<=0)
                        error('Time array must start with values greater than zero')
                    elseif ~(length(time) == length(unique(time)))
                        error('Time elements must not repeat')
                    else
                        if isnumeric(waypoints) && ~isscalar(waypoints)
                            if size(waypoints,1)<4
                                error('Waypoints must be an array with the following format: [position'',speed'',acceleration'',yaw,yaw_derivative'',yaw_sec_derivative'']'', where speed, acceleration, yaw_derivative and yaw_sec_derivative are optional.')
                            else
                                x(1,:) = waypoints(1,:);
                                y(1,:) = waypoints(2,:);
                                z(1,:) = waypoints(3,:);
                                switch size(waypoints,1)
                                    case 4
                                        x(2,:) = zeros(size(waypoints(1,:)));
                                        x(3,:) = x(2,:);
                                        y(2,:) = zeros(size(waypoints(2,:)));
                                        y(3,:) = y(2,:);
                                        z(2,:) = zeros(size(waypoints(3,:)));
                                        z(3,:) = z(2,:);
                                        yaw(1,:) = waypoints(4,:);
                                        yaw(2,:) = zeros(size(waypoints(4,:)));
                                        yaw(3,:) = yaw(2,:);
                                    case 5
                                        x(2,:) = zeros(size(waypoints(1,:)));
                                        x(3,:) = x(2,:);
                                        y(2,:) = zeros(size(waypoints(2,:)));
                                        y(3,:) = y(2,:);
                                        z(2,:) = zeros(size(waypoints(3,:)));
                                        z(3,:) = z(2,:);
                                        yaw(1,:) = waypoints(4,:);
                                        yaw(2,:) = waypoints(5,:);
                                        yaw(3,:) = zeros(size(waypoints(4,:)));
                                    case 6
                                        x(2,:) = zeros(size(waypoints(1,:)));
                                        x(3,:) = x(2,:);
                                        y(2,:) = zeros(size(waypoints(2,:)));
                                        y(3,:) = y(2,:);
                                        z(2,:) = zeros(size(waypoints(3,:)));
                                        z(3,:) = z(2,:);
                                        yaw(1,:) = waypoints(4,:);
                                        yaw(2,:) = waypoints(5,:);
                                        yaw(3,:) = waypoints(5,:);
                                    case 7
                                        x(2,:) = waypoints(4,:);
                                        x(3,:) = zeros(size(waypoints(1,:)));
                                        y(2,:) = waypoints(5,:);
                                        y(3,:) = zeros(size(waypoints(2,:)));
                                        z(2,:) = waypoints(6,:);
                                        z(3,:) = zeros(size(waypoints(3,:)));
                                        yaw(1,:) = waypoints(7,:);
                                        yaw(2,:) = zeros(size(waypoints(7,:)));
                                        yaw(3,:) = yaw(2,:);
                                    case 8
                                        x(2,:) = waypoints(4,:);
                                        x(3,:) = zeros(size(waypoints(1,:)));
                                        y(2,:) = waypoints(5,:);
                                        y(3,:) = zeros(size(waypoints(2,:)));
                                        z(2,:) = waypoints(6,:);
                                        z(3,:) = zeros(size(waypoints(3,:)));
                                        yaw(1,:) = waypoints(7,:);
                                        yaw(2,:) = waypoints(8,:);
                                        yaw(3,:) = zeros(size(waypoints(7,:)));
                                    case 10
                                        x(2,:) = waypoints(4,:);
                                        x(3,:) = waypoints(7,:);
                                        y(2,:) = waypoints(5,:);
                                        y(3,:) = waypoints(8,:);
                                        z(2,:) = waypoints(6,:);
                                        z(3,:) = waypoints(9,:);
                                        yaw(1,:) = waypoints(10,:);
                                        yaw(2,:) = zeros(size(waypoints(10,:)));
                                        yaw(3,:) = yaw(2,:);
                                    case 12
                                        x(2,:) = waypoints(4,:);
                                        x(3,:) = waypoints(7,:);
                                        y(2,:) = waypoints(5,:);
                                        y(3,:) = waypoints(8,:);
                                        z(2,:) = waypoints(6,:);
                                        z(3,:) = waypoints(9,:);
                                        yaw(1,:) = waypoints(10,:);
                                        yaw(2,:) = waypoints(11,:);
                                        yaw(3,:) = waypoints(12,:);
                                    otherwise
                                        error('Waypoints must be an array with the following format: [position'',speed'',acceleration'',yaw,yaw_derivative'',yaw_sec_derivative'']'', where speed, acceleration, yaw_derivative and yaw_sec_derivative are optional.')
                                end  
                                obj.trajectoryMap_ = [];
                                obj.trajectoryMap_.endTime  = [];
                                obj.trajectoryMap_.type     = 'waypoints';
                                obj.trajectoryMap_.ax       = {};
                                obj.trajectoryMap_.ay       = {};
                                obj.trajectoryMap_.az       = {};
                                obj.trajectoryMap_.ayaw     = {};
                                if size(time,2)==1
                                    time = time/size(waypoints,2):time/size(waypoints,2):time;
                                end  
                                if obj.verbose_ == true
                                    disp('Waypoint times set to')
                                    disp(time)
                                end
                                initialAttitude = obj.toEuler(obj.initialState_.attitude);
                                so = [obj.initialState_.position;initialAttitude(3)];
                                dso = [obj.initialState_.velocity;obj.initialState_.angularVelocity(3)];
                                d2so = [0;0;0;0];
                                to = 0;
                                for it=1:size(waypoints,2)
                                    sf = [x(1,it);y(1,it);z(1,it);yaw(1,it)];
                                    dsf = [x(2,it);y(2,it);z(2,it);yaw(2,it)];
                                    d2sf = [x(3,it);y(3,it);z(3,it);yaw(3,it)];
                                    tf = time(it);
                                    obj.trajectoryMap_.endTime(it) = tf;
                                    [obj.trajectoryMap_.ax{it}, obj.trajectoryMap_.ay{it}, obj.trajectoryMap_.az{it}, obj.trajectoryMap_.ayaw{it}] = obj.trajectoryMatrices(to,tf,so,dso,d2so,sf,dsf,d2sf);
                                    so = sf;
                                    dso = dsf;
                                    d2so = d2sf;
                                    to = tf;
                                end
                                if obj.verbose_ == true
                                    disp([num2str(length(obj.trajectoryMap_.ax)),' waypoints configured.'])
                                end
                            end
                        end            
                    end
                otherwise
                    error('Trajectory type not existent!')
            end
        end  
        function configFDD(obj, hitRate, delay)
        %UNTITLED Summary of this function goes here
        %   Detailed explanation goes here
            if isnumeric(hitRate) && isscalar(hitRate) && hitRate>=0.0 && hitRate <= 1.0
                if isnumeric(delay) && isscalar(delay) && delay>=0.0
                    obj.fddConfig_.hitRate = hitRate;
                    obj.fddConfig_.delay = delay;
                    
                    if obj.verbose_ == true
                        disp(['FDD hit rate configured to ',num2str(100*obj.fddConfig_.hitRate),'%.']);
                        disp(['FDD delay configured to ',num2str(obj.fddConfig_.delay),' seconds.']);
                    end
                else
                    error('FDD''s delay must be a numeric scalar greater than or equal to 0.0')
                end
            else
                error('FDD''s hit rate must be a numeric scalar between 0 and 1')
            end
        end  
        function addCommand(obj, commands, time)
        %UNTITLED Summary of this function goes here
        %   Detailed explanation goes here
            classMethods = methods(class(obj));
            if iscellstr(commands)
                if isnumeric(time) && (isequal(size(time),[1 length(commands)]) || isequal(size(time),[length(commands) 1]))
                    for it=1:length(time)
                        commandString = strsplit(commands{it},'(');
                        commandString = commandString{1};
                        if any(strcmp(commandString,classMethods))
                            if isempty(obj.automationConfig_.time)
                                obj.automationConfig_.time(1) = time(1);
                                obj.automationConfig_.commands{1}{1} = commands{1};
                            else
                                index = find(obj.automationConfig_.time==time(it),1);
                                if isempty(index)
                                    obj.automationConfig_.time(end+1) = time(it);
                                    obj.automationConfig_.commands{end+1}{1} = commands{it};
                                else
                                    obj.automationConfig_.commands{index}{end+1} = commands{it};
                                end
                            end
                        else
                            if obj.verbose_ == true
                                warning('Command not recognised. Skipping to next command.')
                            end
                        end
                    end
                    [obj.automationConfig_.time, index] = sort(obj.automationConfig_.time);
                    aux = obj.automationConfig_.commands(index);
                    obj.automationConfig_.commands = aux;
                else
                    error('Time must be a numeric array the same length as the number of commands')
                end
            else
                error('Commands must be a cell array of strings!')
            end
        end
        function setTimeStep(obj, timeStep)
            if isscalar(timeStep) && isnumeric(timeStep) && timeStep>0
                if timeStep>obj.controlTimeStep_/obj.timeStepRelation_
                    warning('Simulation step period greater than %d x (Controller step period)',obj.timeStepRelation_);
                    obj.timeStep_ = obj.controlTimeStep_/obj.timeStepRelation_;
                    if obj.verbose_ == true
                        disp(['Simulation step period changed to ',num2str(obj.timeStep_),' seconds.']);
                    end
                else
                    obj.timeStep_ = timeStep;
                    if obj.verbose_ == true
                        disp(['Simulation step period set to ',num2str(obj.timeStep_),' seconds.']);
                    end
                end
            else
                error('Simulation step period must be scalar and greater than zero');
            end
        end
        function setControlDelay(obj, controlDelay)
            if isscalar(controlDelay) && isnumeric(controlDelay) && controlDelay>0 && controlDelay<1.0
                obj.controlDelay_ = controlDelay;
                if obj.verbose_ == true
                    disp(['Control delay set to ',num2str(obj.controlDelay_*100),'% of control time step.']);
                end
            else
                error('Control delay must be scalar, greater than zero and less than one.');
            end
        end
        function setAngularFilterGain(obj, gain)
            if (isequal(size(gain),[1 3]) || isequal(size(gain),[3 1])) && isnumeric(gain) && all(gain>=0) && all(gain<=1.0)
                obj.filterConfig_.angular = diag(gain);
                if obj.verbose_ == true
                    disp(['Low-pass filter gains for angular velocity set to [',num2str(reshape(gain,[1 3])),'].']);
                end
            else
                error('Low-pass filter gains for angular velocity must be a numeric vector of length 3, ith gains between 0 and 1.');
            end
        end
        function setControlTimeStep(obj, timeStep)
        %UNTITLED Summary of this function goes here
        %   Detailed explanation goes here
            if isscalar(timeStep) && isnumeric(timeStep) && timeStep>0
                obj.controlTimeStep_ = timeStep;
                if obj.verbose_ == true
                    disp(['Controller step period set to ',num2str(obj.controlTimeStep_),' seconds.']);
                end
                if obj.timeStep_>obj.controlTimeStep_/obj.timeStepRelation_                    
                    obj.timeStep_ = obj.controlTimeStep_/obj.timeStepRelation_;
                    if obj.verbose_ == true
                        warning('Simulation step period greater than %d x (Controller step period)',obj.timeStepRelation_);
                        disp(['Simulation step period changed to ',num2str(obj.timeStep_),' seconds.']);
                    end
                end
            else
                error('Controller step period must be scalar and greater than zero');
            end
        end  
        function setRotorDirection(obj, rotorID, direction)
        %UNTITLED Summary of this function goes here
        %   Detailed explanation goes here        
            if iscell(direction)
                warning('Input will be converted to numeric array.')
                direction = cell2mat(direction);
            end
            if isnumeric(direction) && (isequal(size(direction),[1 length(rotorID)]) || isequal(size(direction),[length(rotorID) 1])) && length(rotorID)<=obj.numberOfRotors_
                if (sum((direction==1)+(direction==-1))==length(rotorID))
                    obj.rotorDirection_(rotorID) = direction;
                    if obj.verbose_ == true
                        disp(['Set rotor directions to: [',num2str(reshape(direction,[1 length(rotorID)])),']'])
                    end
                else
                    error('Rotor direction must be equal to 1 or -1.');
                end
            else
                error('Rotor direction must be an array the same length as the vector of rotor IDs, which must not exceed the number of rotors in the multicopter!');
            end
        end
        function setRotorOperatingPoint(obj, rotorID, op)
        %UNTITLED Summary of this function goes here
        %   Detailed explanation goes here        
            if iscell(op)
                warning('Input will be converted to numeric array.')
                op = cell2mat(op);
            end
            if isnumeric(op) && (isequal(size(op),[1 length(rotorID)]) || isequal(size(op),[length(rotorID) 1])) && length(rotorID)<=obj.numberOfRotors_
                obj.rotorOperatingPoint_(rotorID) = op;
                if obj.verbose_ == true
                    disp(['Set rotor operating points to: [',num2str(reshape(op,[1 length(rotorID)])),']'])
                end
            else
                error('Rotor operating point must be an array the same length as the vector of rotor IDs, which must not exceed the number of rotors in the multicopter!');
            end
        end
        function rotorID = addRotor(obj, varargin)
            rotorID = addRotor@multicopter(obj,varargin{:});
            obj.rotorDirection_(rotorID) = 0;
            obj.rotorOperatingPoint_(rotorID) = 750;
            obj.fddRotor_{rotorID}.status{1} = {'free','responding','motor ok','prop ok'};
            obj.fddRotor_{rotorID}.motorEfficiency(1) = 1.0;
            obj.fddRotor_{rotorID}.propEfficiency(1) = 1.0;            
            obj.fddRotor_{rotorID}.time(1) = obj.previousState_.time;
        end
        function removeRotor(obj, varargin)
            removeRotor@multicopter(obj,varargin{:});
            obj.rotorDirection_(varargin{:}) = [];
            obj.rotorOperatingPoint_(varargin{:}) = [];
            obj.fddRotor_(varargin{:}) = [];
        end
        %Get and read methods        
        function output = listControllers(obj)
        %UNTITLED Summary of this function goes here
        %   Detailed explanation goes here
            output = obj.attitudeControllers_;

        end  
        function output = listControlAllocators(obj)
        %UNTITLED Summary of this function goes here
        %   Detailed explanation goes here
            output = obj.controlAllocators_;
        end  
        function listCommands(obj)
        %UNTITLED Summary of this function goes here
        %   Detailed explanation goes here
        aux = 1;
            for it=1:length(obj.automationConfig_.time)
                for j=1:length(obj.automationConfig_.commands{it})
                    disp(['Time: ',num2str(obj.automationConfig_.time(it)),', Command: ',obj.automationConfig_.commands{it}{j}]);
                    aux = aux + 1;
                end
            end
        end  
        function metricsStruct = metrics(obj)
        %UNTITLED Summary of this function goes here
        %   Detailed explanation goes here
            metricsStruct = obj.metrics_;
        end
        function trajectoryStruct = trajectory(obj)
        %UNTITLED Summary of this function goes here
        %   Detailed explanation goes here
            trajectoryStruct = obj.trajectory_;
        end
        function rotorDirectionArray = rotorDirection(obj,varargin)
        %UNTITLED Summary of this function goes here
        %   Detailed explanation goes here
            if isempty(varargin)
                rotorDirectionArray = obj.rotorDirection_;
            else
                rotorDirectionArray = obj.rotorDirection_(rotorID);
            end
        end
        function allProperties = multicontrolProperties(obj)
        %UNTITLED Summary of this function goes here
        %   Detailed explanation goes here
            allProperties.trajectoryMap = obj.trajectoryMap_;      %Map that contains the polynomial trajectory matrices for each axis and period of time
            allProperties.trajectory = obj.trajectory_;         %Struct that contains the desired trajectory in space and attitude reference for each time step
            allProperties.rotorDirection = obj.rotorDirection_;     %Array containing the rotor directions of rotation around the orientation axis. Follows the right-hand rule
            allProperties.rotorOperatingPoint_ = obj.rotorOperatingPoint_;% Array containing the hover operating point velocities for each rotor.
            allProperties.metrics = obj.metrics_;            %Struct containing the metrics to evaluate the controllers performances

            % Simulation settings
            allProperties.controlAlg = obj.controlAlg_;         %Control algorithm used in simulation
            allProperties.allocationAlg = obj.allocationAlg_;      %Control allocation algorithm used in simulation        
            allProperties.attReference = obj.attReferenceAlg_;    %Control allocation algorithm used in attitude reference method

            allProperties.controlTimeStep = obj.controlTimeStep_;    %Time step for the multirotor control. Should not be mistaken with the multirotor simulator time step
            allProperties.controlDelay = obj.controlDelay_;       %Percentage of control time step that the algorithm takes to calculate and apply inputs to multirotor

            % Algorithms configurations
            allProperties.controlConfig = obj.controlConfig_;          %Cell that contains all possible attitude control architectures configurations
            allProperties.positionControlConfig = obj.positionControlConfig_;  %Struct that contains the PIDD position gains
            allProperties.allocationConfig = obj.allocationConfig_;       %Cell that contains all possible control allocation architectures configurations
            allProperties.fddConfig = obj.fddConfig_;              %Struct with FDD rate and delay
            allProperties.automationConfig = obj.automationConfig_;       %Cell that contains the types and intensities of faults and uncertainties at specified periods  
            allProperties.filterConfig = obj.filterConfig_;
        end
        %Simulation commands
        function clearCommands(obj)
        %UNTITLED Summary of this function goes here
        %   Detailed explanation goes here
            obj.automationConfig_.time      = [];
            obj.automationConfig_.commands  = {};             
        end  
        function run(obj, varargin)
        %UNTITLED Summary of this function goes here
        %   Detailed explanation goes here
            if obj.canRunControl()
                defaultMetricPrecision = 0.1;
                defaultAngularPrecision = 5;
                defaultVisualizeGraph = false;
                defaultEndTime = 1.3*obj.trajectoryMap_.endTime(end);
                defaultError = 5;

                p = inputParser;
                validScalarPosNum = @(x) isnumeric(x) && isscalar(x) && (x > 0);
                validBoolean = @(x) isscalar(x) && (x==true || x==false);
                addParameter(p,'metricPrecision',defaultMetricPrecision,validScalarPosNum);
                addParameter(p,'angularPrecision',defaultAngularPrecision,validScalarPosNum);
                addParameter(p,'visualizeGraph',defaultVisualizeGraph,validBoolean);
                addParameter(p,'visualizeProgress',defaultVisualizeGraph,validBoolean);
                addParameter(p,'endTime',defaultEndTime,@(x) isnumeric(x) && isscalar(x) && (x >= obj.trajectoryMap_.endTime(end)));
                addParameter(p,'endError',defaultError,validScalarPosNum);
                parse(p,varargin{:});
                
                obj.clearTrajectory();
                obj.clearMetrics();
                obj.clearFDD();
                
                obj.metrics_.simulationEndError = p.Results.endError;
                obj.metrics_.missionMetricPrecision = p.Results.metricPrecision;
                obj.metrics_.missionAngularPrecision = p.Results.angularPrecision;
                visualizeGraph = p.Results.visualizeGraph;
                visualizeProgress = p.Results.visualizeProgress;
                endTime = p.Results.endTime;
                endError = p.Results.endError;
                    
                obj.clearLog();
                obj.startLogging();
                if obj.isRunning()
                    obj.reset();
                end
                
                currentTime = 0;
                dt = obj.controlTimeStep_;
                %endTime = obj.trajectoryMap_.endTime(end);
                
                obj.trajectory_.time(end+1) = currentTime;
                obj.trajectory_.position(:,end+1) = obj.initialPosition();
                obj.trajectory_.velocity(:,end+1) = obj.initialVelocity();   
                obj.trajectory_.acceleration(:,end+1) = obj.initialAcceleration();
                obj.trajectory_.attitude(:,end+1) = obj.initialAttitude();
                obj.trajectory_.angularVelocity(:,end+1) = obj.initialAngularVelocity();
                          
                if visualizeGraph==true
                    figure      
                    l_x = 0.4;
                    l_y = 0.2;
                    l_z = 0.1;
                    corpo1.vertices(1,:) = [l_x, 0, 0]; 
                    corpo1.vertices(2,:) = [0, l_y, 0]; 
                    corpo1.vertices(3,:) = [0,-l_y, 0]; 
                    corpo1.vertices(4,:) = [0, 0, l_z]; 
                    corpo1.vertices(5,:) = [0, 0,-l_z];
                    corpo1.vertices(6,:) = [-l_x, 0, 0]; 
                    corpo1.faces = [1 2 5; 1 3 5; 1 3 4; 1 2 4; 6 2 5; 6 3 5; 6 3 4];
                    corpo1.n = size(corpo1.vertices,1);
                    subplot(2,4,1)
                    corpo1.objeto = patch('Vertices',corpo1.vertices,...
                        'Faces',corpo1.faces,'FaceColor','blue');
                    corpo2 = corpo1;
                    subplot(2,4,5)
                    corpo2.objeto = patch('Vertices',corpo2.vertices,...
                        'Faces',corpo2.faces,'FaceColor','blue');
                end
                if visualizeProgress==true
                    disp('Progress: 0 %')
                end
                % Incrents simulation time
                currentTime = currentTime + dt;
                positionError = 0;
                obj.log_.diagnosis = {};
                while currentTime<=endTime && positionError<=endError
                    % Calculates desired state for the current time based
                    % on desired trajectory
                    obj.trajectory_.time(end+1) = currentTime;
                    [desiredState.position,...
                     desiredState.velocity,...
                     desiredState.acceleration,...
                     auxYaw] = obj.desiredTrajectory(currentTime);
                    % Saves trajectory point
                    obj.trajectory_.position(:,end+1) = desiredState.position;
                    obj.trajectory_.velocity(:,end+1) = desiredState.velocity;
                    obj.trajectory_.acceleration(:,end+1) = desiredState.acceleration;
                    % Diagnoses any fault occured during last simulations
                    diagnosis = obj.fdd(currentTime);
                    obj.log_.diagnosis{end+1} = diagnosis;
                    % Calculates position control impulses
                    controllerOutput.positionController = obj.positionControl(desiredState);
                    % Calculates attitude reference for torque control
                    desiredAttitude = attitudeReference(obj, controllerOutput.positionController, auxYaw(1), diagnosis); 
                    obj.trajectory_.attitude(:,end+1) = desiredAttitude;
                    % Calculates attitude control output
                    controllerOutput.attitudeController = obj.control(desiredAttitude,controllerOutput.positionController,diagnosis);
                    % Allocates torques to actuators in case there's
                    % control allocation 
                    actuatorInput = obj.controlAllocation(controllerOutput, diagnosis);
                    % Executes automated commands
                    commandsIndex = find(obj.automationConfig_.time<currentTime & obj.automationConfig_.time>=obj.previousTime());
                    obj.evalCommands(commandsIndex);
                    % Simulates aircraft
                    inputTime = currentTime-(1-obj.controlDelay_)*dt;
                    [t, output] = run@multicopter(obj,actuatorInput,[inputTime,currentTime]);
                    if visualizeGraph==true                      
                        log = obj.log();
                        %Update Body plots
                        x=log.position(1,end);
                        y=log.position(2,end);
                        z=log.position(3,end);
                        eulerAngles = obj.toEuler(log.attitude(:,end));
                        theta_r=eulerAngles(1)*180/pi;
                        theta_p=eulerAngles(2)*180/pi;
                        theta_y=eulerAngles(3)*180/pi;
                        eulerVector = (180/pi).*obj.toEuler(log.attitude);
                        rollVector = eulerVector(1,:);
                        pitchVector = eulerVector(2,:);
                        yawVector = eulerVector(3,:);
                        
                        %Update Linear plots
                        % 3D visualization 1
                        subplot(2,4,1)  
                        grid on
                        title('3D visualization');
                        xlabel('X [m]');
                        ylabel('Y [m]');
                        zlabel('Z [m]');
                        view([45,30])
                        xlim([-3 3])
                        ylim([-3 3])
                        zlim([-3 3])
                        obj.moveObjectDisplay(corpo1,[theta_r;theta_p;theta_y],[x;y;z]);
                        hold on
                        plot3(log.position(1,:),log.position(2,:),log.position(3,:))
                        % 3D visualization 2
                        subplot(2,4,5)
                        grid on
                        title('3D visualization');
                        xlabel('X [m]');
                        ylabel('Y [m]');
                        zlabel('Z [m]');
                        view([0,90])
                        xlim([-3 3])
                        ylim([-3 3])
                        zlim([-3 3])
                        obj.moveObjectDisplay(corpo2,[theta_r;theta_p;theta_y],[x;y;z]);
                        hold on
                        plot3(log.position(1,:),log.position(2,:),log.position(3,:))         
                        % X position
                        subplot(2,4,2)
                        grid on
                        title('X Position');
                        xlabel('time [s]');
                        ylabel('X [m]');
                        plot(obj.trajectory_.time,obj.trajectory_.position(1,:),'--g') 
                        hold on
                        plot(log.time,log.position(1,:),'b')
                        legend('Desired','Executed');
                        % Y Position
                        subplot(2,4,3)
                        grid on
                        title('Y Position');
                        xlabel('time [s]');
                        ylabel('Y [m]');
                        plot(obj.trajectory_.time,obj.trajectory_.position(2,:),'--g')
                        hold on
                        plot(log.time,log.position(2,:),'b')
                        legend('Desired','Executed');
                        % Z Position
                        subplot(2,4,4)
                        grid on
                        title('Z Position');
                        xlabel('time [s]');
                        ylabel('Z [m]');
                        plot(obj.trajectory_.time,obj.trajectory_.position(3,:),'--g')
                        hold on
                        plot(log.time,log.position(3,:),'b')
                        legend('Desired','Executed');
                        
                        %Update Angular plots
                        trajectoryEulerAttitude = (180/pi).*obj.toEuler(obj.trajectory_.attitude);                       
                        % Roll angle
                        subplot(2,4,6)
                        grid on
                        title('Roll angle');
                        xlabel('time [s]');
                        ylabel('Roll [�]');
                        plot(obj.trajectory_.time,trajectoryEulerAttitude(1,:),'--g')
                        hold on
                        plot(log.time,rollVector,'b')
                        legend('Desired','Executed');
                        % Pitch angle
                        subplot(2,4,7)
                        grid on
                        title('Pitch angle');
                        xlabel('time [s]');
                        ylabel('Pitch [�]');
                        plot(obj.trajectory_.time,trajectoryEulerAttitude(2,:),'--g')
                        hold on
                        plot(log.time,pitchVector,'b')
                        legend('Desired','Executed');
                        % Yaw angle
                        subplot(2,4,8)
                        grid on
                        title('Yaw angle');
                        xlabel('time [s]');
                        ylabel('Yaw [�]');
                        plot(obj.trajectory_.time,trajectoryEulerAttitude(3,:),'--g') 
                        hold on
                        plot(log.time,yawVector,'b')
                        legend('Desired','Executed');
                    end                    
                    % Incrents simulation time
                    if visualizeProgress==true && rem(round(100*currentTime/endTime),5)==0
                        disp(['Progress: ',num2str(100*currentTime/endTime),' %'])
                    end
                    currentTime = currentTime + dt;
                    positionError = sqrt(sum((obj.log_.position(:,end)-obj.trajectory_.position(:,end)).^2,1));
                end
                if positionError>endError || isnan(positionError)
                    if obj.verbose_ == true
                        disp('Simulation terminated. Position error too large, indicating simulation divergence')
                    end
                    obj.metrics_.simulationSuccess = 1-abs(endTime-currentTime)/endTime;
                    obj.updateMetrics();
                else
                    if obj.verbose_ == true
                        disp('Simulation ended normaly.')
                    end
                    obj.metrics_.simulationSuccess = 1;
                    obj.updateMetrics();
                end
            else
                error('Cannot Run. System not set correctly.')
            end
        end   
        function save(obj, type, varargin)
        %UNTITLED Summary of this function goes here
        %   Detailed explanation goes here
            switch type
                case 'all'
                    controller = struct(obj);
                    if isempty(varargin)
                        [filename, pathname] = uiputfile('simulation.mat');
                        save([pathname,filename],'-struct','controller');
                    else
                        save(varargin{1},'-struct','controller');
                    end
                case 'result'
                    log_ = obj.log_;
                    trajectory_ = obj.trajectory_;
                    metrics_ = obj.metrics_;
                    if isempty(varargin)
                        [filename, pathname] = uiputfile('simulation_results.mat');
                        save([pathname,filename],'log_','trajectory_','metrics_');
                    else
                        save(varargin{1},'log_','trajectory_','metrics_');
                    end
                case 'trajectory'
                    trajectoryMap_ = obj.trajectoryMap_;
                    automationConfig_ = obj.automationConfig_;
                    if isempty(varargin)
                        [filename, pathname] = uiputfile('trajectoryMap.mat');
                        save([pathname,filename],'trajectoryMap_','automationConfig_');
                    else
                        save(varargin{1},'trajectoryMap_','automationConfig_');
                    end
                case 'controller'
                    rotorDirection_ = obj.rotorDirection_;
                    rotorOperatingPoint_ = obj.rotorOperatingPoint_;
                    controlAlg_ = obj.controlAlg_;
                    allocationAlg_ = obj.allocationAlg_;
                    attReferenceAlg_ = obj.attReferenceAlg_;
                    controlTimeStep_ = obj.controlTimeStep_;
                    controlDelay_ = obj.controlDelay_;
                    controlConfig_ = obj.controlConfig_;
                    positionControlConfig_ = obj.positionControlConfig_;
                    allocationConfig_ = obj.allocationConfig_;
                    fddConfig_ = obj.fddConfig_;
                    automationConfig_ = obj.automationConfig_;
                    timeStepRelation_ = obj.timeStepRelation_;
                    
                    if isempty(varargin)
                        [filename, pathname] = uiputfile('controller.mat');
                        save([pathname,filename],'rotorDirection_','rotorOperatingPoint_',...
                            'controlAlg_','allocationAlg_','attReferenceAlg_',...
                            'controlTimeStep_','controlDelay_','controlConfig_',...
                            'positionControlConfig_','allocationConfig_','fddConfig_',...
                            'automationConfig_','timeStepRelation_');
                    else
                        save(varargin{1},'rotorDirection_','rotorOperatingPoint_',...
                            'controlAlg_','allocationAlg_','attReferenceAlg_',...
                            'controlTimeStep_','controlDelay_','controlConfig_',...
                            'positionControlConfig_','allocationConfig_','fddConfig_',...
                            'automationConfig_','timeStepRelation_');
                    end
                case 'multirotor'
                    mass_ = obj.mass_;
                    inertiaTensor_ = obj.inertiaTensor_;
                    translationalFriction_ = obj.translationalFriction_;
                    numberOfRotors_ = obj.numberOfRotors_;
                    rotor_ = obj.rotor_;
                    massPError_ = obj.massPError_;
                    translationalFrictionPError_ = obj.translationalFrictionPError_;
                    inertiaTensorPError_ = obj.inertiaTensorPError_;
                    cgPositionError_ = obj.cgPositionError_;
                    timeStep_ = obj.timeStep_;
                    initialState_ = obj.initialState_;
                    isLogging_ = obj.isLogging_;
                    opts_ = obj.opts_;
                    simEffects_ = obj.simEffects_;
                    solver_ = obj.solver_;
                    linearDisturbance_ = obj.linearDisturbance_;
                    verbose_ = obj.verbose_;
                    
                    if isempty(varargin)
                        [filename, pathname] = uiputfile('multirotor.mat');
                        save([pathname,filename],'mass_','inertiaTensor_','translationalFriction_',...
                            'numberOfRotors_','rotor_','massPError_','translationalFrictionPError_',...
                            'inertiaTensorPError_','cgPositionError_','timeStep_',...
                            'initialState_','isLogging_','opts_','simEffects_',...
                            'solver_','linearDisturbance_','verbose_');
                    else
                        save(varargin{1},'mass_','inertiaTensor_','translationalFriction_',...
                            'numberOfRotors_','rotor_','massPError_','translationalFrictionPError_',...
                            'inertiaTensorPError_','cgPositionError_','timeStep_',...
                            'initialState_','isLogging_','opts_','simEffects_',...
                            'solver_','linearDisturbance_','verbose_');
                    end
                otherwise
                    if strcmp(type,'figures') || strcmp(type,'graphs')
                        if isempty(varargin)
                            pathName = uigetdir();
                        else
                            pathName = varargin{1};
                        end
                        if pathName~=0
                            % X Position
                            xAx = figure();
                            obj.plotAxes('xPosition',xAx);
                            % Y Position
                            yAx = figure();
                            obj.plotAxes('yPosition',yAx);
                            % Z Position
                            zAx = figure();
                            obj.plotAxes('zPosition',zAx);
                            % Z Position
                            positionAx = figure();
                            obj.plotAxes('position',positionAx);
                            % X Speed
                            xSpeedAx = figure();
                            obj.plotAxes('xSpeed',xSpeedAx);
                            % Y Speed
                            ySpeedAx = figure();
                            obj.plotAxes('ySpeed',ySpeedAx);
                            % Z Speed
                            zSpeedAx = figure();
                            obj.plotAxes('zSpeed',zSpeedAx);
                            % Overall Speed
                            speedAx = figure();
                            obj.plotAxes('speed',speedAx);
                            % Roll angle
                            rollAx = figure();
                            obj.plotAxes('roll',rollAx);
                            % Pitch angle
                            pitchAx = figure();
                            obj.plotAxes('pitch',pitchAx);
                            % Yaw angle
                            yawAx = figure();
                            obj.plotAxes('yaw',yawAx);
                            % Roll Speed
                            rollSpeedAx = figure();
                            obj.plotAxes('rollSpeed',rollSpeedAx);
                            % Pitch Speed
                            pitchSpeedAx = figure();
                            obj.plotAxes('pitchSpeed',pitchSpeedAx);
                            % Yaw Speed
                            yawSpeedAx = figure();
                            obj.plotAxes('yawSpeed',yawSpeedAx);
                            % Rotor speeds
                            rSpeedAx = [];
                            for it=1:length(obj.log_.rotor)
                                rSpeedAx = [rSpeedAx, figure()];
                            end
                            obj.plotAxes('rotorSpeed',rSpeedAx);
                            % Rotor Acelerations
                            rAcelerationAx = [];
                            for it=1:length(obj.log_.rotor)
                                rAcelerationAx = [rAcelerationAx, figure()];
                            end
                            obj.plotAxes('rotorAcceleration',rAcelerationAx);
                            % Rotor torques
                            rTorqueAx = [];
                            for it=1:length(obj.log_.rotor)
                                rTorqueAx = [rTorqueAx, figure()];
                            end
                            obj.plotAxes('rotorTorque',rTorqueAx);
                            % Mechanical power
                            powerAx = figure();
                            obj.plotAxes('power',powerAx);
                            switch type
                                case 'figures'
                                    saveas(xAx,[pathName,'\xposition']);
                                    saveas(yAx,[pathName,'\yposition']);
                                    saveas(zAx,[pathName,'\zposition']);
                                    saveas(positionAx,[pathName,'\position']);
                                    saveas(xSpeedAx,[pathName,'\xvelocity']);
                                    saveas(ySpeedAx,[pathName,'\yvelocity']);
                                    saveas(zSpeedAx,[pathName,'\zvelocity']);
                                    saveas(speedAx,[pathName,'\aircraftspeed']);
                                    saveas(rollAx,[pathName,'\roll']);
                                    saveas(pitchAx,[pathName,'\pitch']);
                                    saveas(yawAx,[pathName,'\yaw']);
                                    saveas(rollSpeedAx,[pathName,'\rollvelocity']);
                                    saveas(pitchSpeedAx,[pathName,'\pitchvelocity']);
                                    saveas(yawSpeedAx,[pathName,'\yawvelocity']);
                                    for it=1:length(obj.log_.rotor)
                                        saveas(rSpeedAx(it),[pathName,'\rotor',num2str(it),'speed']);
                                        saveas(rAcelerationAx(it),[pathName,'\rotor',num2str(it),'acceleration']);
                                        saveas(rTorqueAx(it),[pathName,'\rotor',num2str(it),'torque']);
                                    end
                                    saveas(powerAx,[pathName,'\power']);
                                case 'graphs'
                                    saveas(xAx,[pathName,'\xposition.png']);
                                    saveas(yAx,[pathName,'\yposition.png']);
                                    saveas(zAx,[pathName,'\zposition.png']);
                                    saveas(positionAx,[pathName,'\position.png']);
                                    saveas(xSpeedAx,[pathName,'\xspeed.png']);
                                    saveas(ySpeedAx,[pathName,'\yspeed.png']);
                                    saveas(zSpeedAx,[pathName,'\zspeed.png']);
                                    saveas(speedAx,[pathName,'\aircraftspeed.png']);
                                    saveas(rollAx,[pathName,'\roll.png']);
                                    saveas(pitchAx,[pathName,'\pitch.png']);
                                    saveas(yawAx,[pathName,'\yaw.png']);
                                    saveas(rollSpeedAx,[pathName,'\rollspeed.png']);
                                    saveas(pitchSpeedAx,[pathName,'\pitchspeed.png']);
                                    saveas(yawSpeedAx,[pathName,'\yawspeed.png']);
                                    for it=1:length(obj.log_.rotor)
                                        saveas(rSpeedAx(it),[pathName,'\rotor',num2str(it),'speed.png']);
                                        saveas(rAcelerationAx(it),[pathName,'\rotor',num2str(it),'acceleration.png']);
                                        saveas(rTorqueAx(it),[pathName,'\rotor',num2str(it),'torque.png']);
                                    end
                                    saveas(powerAx,[pathName,'\power.png']);
                            end
%                             close all hidden
                            close(xAx)
                            close(yAx)
                            close(zAx)
                            close(positionAx)
                            close(xSpeedAx)
                            close(ySpeedAx)
                            close(zSpeedAx)
                            close(speedAx)
                            close(rollAx)
                            close(pitchAx)
                            close(yawAx)
                            close(rollSpeedAx)
                            close(pitchSpeedAx)
                            close(yawSpeedAx)
                            close(rSpeedAx)
                            close(rAcelerationAx)
                            close(rTorqueAx)
                            close(powerAx)
                        end
                    end                                        
            end
        end  
        function load(obj, varargin)
        %UNTITLED Summary of this function goes here
        %   Detailed explanation goes here
            if isempty(varargin)
                [filename,pathname] = uigetfile;
                pathname = [pathname,filename];
            else
                pathname = varargin{1};
            end
            if ~isempty(pathname) & pathname~=0
                object = load(pathname);
                names = fieldnames(object);
                for it=1:length(names)
                    eval(['obj.',names{it},'=object.',names{it},';']);
                end 
                if obj.verbose_ == true
                    disp('The following variables were loaded to the model:')
                    disp(names)
                end
            end
        end  
        function ax = plotSim(obj)
        %UNTITLED Summary of this function goes here
        %   Detailed explanation goes here
            ax.figure = figure();
            tgp = uitabgroup(ax.figure);
            tab1 = uitab(tgp,'Title','Results');
            axes(tab1);
            ax.xAx = subplot(2,4,1);
            ax.yAx = subplot(2,4,2);
            ax.zAx = subplot(2,4,3);
            ax.rollAx = subplot(2,4,5);
            ax.pitchAx = subplot(2,4,6);
            ax.yawAx = subplot(2,4,7);
            ax.inputAx = subplot(2,4,8);
            ax.speedAx = subplot(2,4,4);            

            obj.plotAxes('xPosition',ax.xAx);
            obj.plotAxes('yPosition',ax.yAx);
            obj.plotAxes('zPosition',ax.zAx);
            
            obj.plotAxes('roll',ax.rollAx);
            obj.plotAxes('pitch',ax.pitchAx);
            obj.plotAxes('yaw',ax.yawAx);
            
            obj.plotAxes('power',ax.inputAx);
            obj.plotAxes('speed',ax.speedAx);
            
            tab2 = uitab(tgp,'Title','3D PLOT');
            ax.positionAx = axes(tab2);
            obj.plotAxes('position',ax.positionAx)
        end
        function plotAxes(obj,varargin)
        %UNTITLED Summary of this function goes here
        %   Detailed explanation goes here
            default = [];

            p = inputParser;
            validAxes = @(x) ishandle(x);
            validAxesArray = @(x) all(ishandle(x));
            addParameter(p,'xPosition',default,validAxes);
            addParameter(p,'yPosition',default,validAxes);
            addParameter(p,'zPosition',default,validAxes);
            addParameter(p,'position',default,validAxes);
            addParameter(p,'xSpeed',default,validAxes);
            addParameter(p,'ySpeed',default,validAxes);
            addParameter(p,'zSpeed',default,validAxes);
            addParameter(p,'speed',default,validAxes);
            addParameter(p,'roll',default,validAxes);
            addParameter(p,'pitch',default,validAxes);
            addParameter(p,'yaw',default,validAxes);
            addParameter(p,'rollSpeed',default,validAxes);
            addParameter(p,'pitchSpeed',default,validAxes);
            addParameter(p,'yawSpeed',default,validAxes);
            addParameter(p,'rotorSpeed',default,validAxesArray);
            addParameter(p,'rotorAcceleration',default,validAxesArray);
            addParameter(p,'rotorTorque',default,validAxesArray);
            addParameter(p,'power',default,validAxes);
            parse(p,varargin{:});

            xAx = p.Results.xPosition;
            yAx = p.Results.yPosition;
            zAx = p.Results.zPosition;
            positionAx = p.Results.position;
            xSpeedAx = p.Results.xSpeed;
            ySpeedAx = p.Results.ySpeed;
            zSpeedAx = p.Results.zSpeed;
            speedAx = p.Results.speed;
            rollAx = p.Results.roll;
            pitchAx = p.Results.pitch;
            yawAx = p.Results.yaw;
            rollSpeedAx = p.Results.rollSpeed;
            pitchSpeedAx = p.Results.pitchSpeed;
            yawSpeedAx = p.Results.yawSpeed;
            rSpeedAx = p.Results.rotorSpeed;
            rAccelerationAx = p.Results.rotorAcceleration;
            rTorqueAx = p.Results.rotorTorque;
            powerAx = p.Results.power;
            
            if ~isempty(xAx)
                axes(xAx)
                grid on
                plot(obj.trajectory_.time,obj.trajectory_.position(1,:),'--');
                hold on
                plot(obj.log_.time,obj.log_.position(1,:));
                title('X Position');
                xlabel('time [s]');
                ylabel('X [m]');
                legend('Desired','Executed','location','best');
            end
            if ~isempty(yAx)
                axes(yAx)
                grid on
                plot(obj.trajectory_.time,obj.trajectory_.position(2,:),'--');
                hold on
                plot(obj.log_.time,obj.log_.position(2,:));
                title('Y Position');
                xlabel('time [s]');
                ylabel('Y [m]');
                legend('Desired','Executed','location','best');
            end
            if ~isempty(zAx)
                axes(zAx)
                grid on
                plot(obj.trajectory_.time,obj.trajectory_.position(3,:),'--');
                hold on
                plot(obj.log_.time,obj.log_.position(3,:));
                title('Z Position');
                xlabel('time [s]');
                ylabel('Z [m]');
                legend('Desired','Executed','location','best');
            end
            if ~isempty(positionAx)
                axes(positionAx)
                grid on
                plot3(obj.trajectory_.position(1,:),obj.trajectory_.position(2,:),obj.trajectory_.position(3,:),'--');
                hold on
                plot3(obj.log_.position(1,:),obj.log_.position(2,:),obj.log_.position(3,:));
                daspect([1 1 1])
                grid on
                title('3D Position');
                xlabel('X Position [m]');
                ylabel('Y Position [m]');
                zlabel('Z Position [m]');
                legend('Desired','Executed','location','best');
            end
            if ~isempty(xSpeedAx)
                axes(xSpeedAx)
                grid on
                plot(obj.trajectory_.time,obj.trajectory_.velocity(1,:),'--');
                hold on
                plot(obj.log_.time,obj.log_.velocity(1,:));
                title('X Velocity');
                xlabel('time [s]');
                ylabel('Vx [m/s]');
                legend('Desired','Executed','location','best');
            end
            if ~isempty(ySpeedAx)
                axes(ySpeedAx)
                grid on
                plot(obj.trajectory_.time,obj.trajectory_.velocity(2,:),'--');
                hold on
                plot(obj.log_.time,obj.log_.velocity(2,:));
                title('Y Velocity');
                xlabel('time [s]');
                ylabel('Vy [m/s]');
                legend('Desired','Executed','location','best');
            end
            if ~isempty(zSpeedAx)
                axes(zSpeedAx)
                grid on
                plot(obj.trajectory_.time,obj.trajectory_.velocity(3,:),'--');
                hold on
                plot(obj.log_.time,obj.log_.velocity(3,:));
                title('Z Velocity');
                xlabel('time [s]');
                ylabel('Vz [m/s]');
                legend('Desired','Executed','location','best');
            end
            if ~isempty(speedAx)
                axes(speedAx)
                grid on
                speed = sqrt(sum(obj.log_.velocity.^2,1));
                desiredSpeed = sqrt(sum(obj.trajectory_.velocity.^2,1));
                plot(obj.trajectory_.time,desiredSpeed,'--');
                hold on
                plot(obj.log_.time,speed);
                title('Aircraft''s overall speed');
                xlabel('time [s]');
                ylabel('Speed [m/s]');
                legend('Desired','Executed','location','best');
            end       
            eulerVector = (180/pi).*obj.toEuler(obj.log_.attitude);
            trajectoryEulerAttitude = (180/pi).*obj.toEuler(obj.trajectory_.attitude);
            if ~isempty(rollAx)
                axes(rollAx)
                grid on
                plot(obj.trajectory_.time,trajectoryEulerAttitude(1,:),'--');
                hold on
                plot(obj.log_.time,eulerVector(1,:));
                title('Roll angle');
                xlabel('time [s]');
                ylabel('Roll [�]');
                legend('Desired','Executed','location','best');                
            end
            if ~isempty(pitchAx)
                axes(pitchAx)
                grid on
                plot(obj.trajectory_.time,trajectoryEulerAttitude(2,:),'--');
                hold on
                plot(obj.log_.time,eulerVector(2,:));
                title('Pitch angle');
                xlabel('time [s]');
                ylabel('Pitch [�]');
                legend('Desired','Executed','location','best');                
            end
            if ~isempty(yawAx)
                axes(yawAx)
                grid on
                plot(obj.trajectory_.time,trajectoryEulerAttitude(3,:),'--');
                hold on
                plot(obj.log_.time,eulerVector(3,:));
                title('Yaw angle');
                xlabel('time [s]');
                ylabel('Yaw [�]');
                legend('Desired','Executed','location','best');                
            end
            if ~isempty(rollSpeedAx)
                axes(rollSpeedAx)
                grid on
                if ~isempty(obj.trajectory_.angularVelocity)
                    plot(obj.trajectory_.time,obj.trajectory_.angularVelocity(1,:),'--');
                    hold on
                end
                plot(obj.log_.time,obj.log_.angularVelocity(1,:));
                title('Roll speed');
                xlabel('time [s]');
                ylabel('Roll speed [rad/s]');
                if ~isempty(obj.trajectory_.angularVelocity)
                    legend('Desired','Executed','location','best');
                else
                    legend('Desired','location','best');
                end
            end
            if ~isempty(pitchSpeedAx)
                axes(pitchSpeedAx)
                grid on
                if ~isempty(obj.trajectory_.angularVelocity)
                    plot(obj.trajectory_.time,obj.trajectory_.angularVelocity(2,:),'--');
                    hold on
                end
                plot(obj.log_.time,obj.log_.angularVelocity(2,:));
                title('Pitch speed');
                xlabel('time [s]');
                ylabel('Pitch speed [rad/s]');
                if ~isempty(obj.trajectory_.angularVelocity)
                    legend('Desired','Executed','location','best');
                else
                    legend('Desired','location','best');
                end
            end
            if ~isempty(yawSpeedAx)
                axes(yawSpeedAx)
                grid on
                if ~isempty(obj.trajectory_.angularVelocity)
                    plot(obj.trajectory_.time,obj.trajectory_.angularVelocity(3,:),'--');
                    hold on
                end
                plot(obj.log_.time,obj.log_.angularVelocity(3,:));
                title('Yaw speed');
                xlabel('time [s]');
                ylabel('Yaw speed [rad/s]');
                if ~isempty(obj.trajectory_.angularVelocity)
                    legend('Desired','Executed','location','best');
                else
                    legend('Desired','location','best');
                end
            end
            for it=1:min(length(rSpeedAx),length(obj.log_.rotor))
                if isgraphics(rSpeedAx(it),'figure')
                    figure(rSpeedAx(it))
                    %set(rSpeedAx(it),'Visible','Off')
                else
                    axes(rSpeedAx(it))
                end
                grid on
                plot(obj.log_.time,obj.log_.input(it,:),'--');
                hold on
                plot(obj.log_.time,obj.log_.rotor(it).setPoint,'--');
                hold on
                plot(obj.log_.time,obj.log_.rotor(it).speed);
                title(['Rotor ',num2str(it),' speed and input']);
                xlabel('time [s]');
                ylabel('Speed [rad/s]');
                legend('Input from controller','Motor control set point','Rotor speed','location','best');
                drawnow
            end
            for it=1:min(length(rAccelerationAx),length(obj.log_.rotor))
                if ~isempty(obj.log_.rotor(it).acceleration)
                    if isgraphics(rAccelerationAx(it),'figure')
                        figure(rAccelerationAx(it))
                        %set(rAccelerationAx(it),'Visible','Off')
                    else
                        axes(rAccelerationAx(it))
                    end
                    grid on
                    plot(obj.log_.time,obj.log_.rotor(it).acceleration);
                    title(['Rotor ',num2str(it),' acceleration']);
                    xlabel('time [s]');
                    ylabel('Acceleration [rad/s]');
                    legend('Rotor acceleration','location','best');
                end
            end
            for it=1:min(length(rTorqueAx),length(obj.log_.rotor))
                if isgraphics(rTorqueAx(it),'figure')
                    figure(rTorqueAx(it))
                    %set(rTorqueAx(it),'Visible','Off')
                else
                    axes(rTorqueAx(it))
                end
                grid on
                plot(obj.log_.time,obj.log_.rotor(it).torque);
                title(['Rotor ',num2str(it),' torque']);
                xlabel('time [s]');
                ylabel('Torque [N.m]');
                legend('Rotor torque','location','best');
            end
            if ~isempty(powerAx)
                axes(powerAx)
                grid on
                plot(obj.log_.time,obj.log_.power);
                title('Power without losses');
                xlabel('time [s]');
                ylabel('Power [W]');
                legend('Total power','location','best');
            end
        end
    end
    
    %%% Private Methods
    methods (Access = private)
        %Controller methods
        function diagnosis = fdd(obj, currentTime)
        %UNTITLED Summary of this function goes here
        %   Detailed explanation goes here     
            if ~(isempty(obj.fddConfig_.hitRate) || isempty(obj.fddConfig_.delay))
                %rouletteAux = [ones(1,uint64(10000*obj.fddConfig_.hitRate)),zeros(1,uint64(10000*(1-obj.fddConfig_.hitRate)))];
                for it=1:obj.numberOfRotors_                
                    obj.fddRotor_{it}.status{end+1} = obj.rotor_(it).status;
                    obj.fddRotor_{it}.motorEfficiency(end+1) = obj.rotor_(it).motorEfficiency;
                    obj.fddRotor_{it}.propEfficiency(end+1) = obj.rotor_(it).propEfficiency;            
                    obj.fddRotor_{it}.time(end+1) = obj.previousState_.time;
                    for j=1:length(obj.fddRotor_{it}.time)-1
                        if obj.fddRotor_{it}.time(1)<obj.previousState_.time-obj.fddConfig_.delay                    
                            obj.fddRotor_{it}.status(1) = [];
                            obj.fddRotor_{it}.motorEfficiency(1) = [];
                            obj.fddRotor_{it}.propEfficiency(1) = [];            
                            obj.fddRotor_{it}.time(1) = [];
                        end
                    end 
                    diagnosis{it}.status{1} = obj.fddRotor_{it}.status{1}{1};
                    diagnosis{it}.status{2} = obj.fddRotor_{it}.status{1}{2};
                    diagnosis{it}.status{3} = obj.fddRotor_{it}.status{1}{3};
                    diagnosis{it}.status{4} = obj.fddRotor_{it}.status{1}{4};
                    error = (1-obj.fddConfig_.hitRate)*randn();
                    test1 = obj.fddRotor_{it}.motorEfficiency(1)+error;
                    if test1>=1
                        test1 = 1;
                    elseif test1 <= 0 
                        test1 = 0;
                    end
                    test2 = obj.fddRotor_{it}.motorEfficiency(1)-error;
                    if test2>=1
                        test2 = 1;
                    elseif test2 <= 0 
                        test2 = 0;
                    end
                    if abs(obj.fddRotor_{it}.motorEfficiency(1)-test1) >= abs(obj.fddRotor_{it}.motorEfficiency(1)-test2)
                        diagnosis{it}.motorEfficiency = test1;
                    else
                        diagnosis{it}.motorEfficiency = test2;
                    end
                    test1 = obj.fddRotor_{it}.propEfficiency(1)+error;
                    if test1>=1
                        test1 = 1;
                    elseif test1 <= 0 
                        test1 = 0;
                    end
                    test2 = obj.fddRotor_{it}.propEfficiency(1)-error;
                    if test2>=1
                        test2 = 1;
                    elseif test2 <= 0 
                        test2 = 0;
                    end
                    if abs(obj.fddRotor_{it}.propEfficiency(1)-test1) >= abs(obj.fddRotor_{it}.propEfficiency(1)-test2)
                        diagnosis{it}.propEfficiency = test1;
                    else
                        diagnosis{it}.propEfficiency = test2;
                    end
                    
                    if diagnosis{it}.motorEfficiency >= 0.7
                        diagnosis{it}.status{3} = 'motor ok';
                    else
                        diagnosis{it}.status{3} = 'motor loss';
                    end                        
                    if diagnosis{it}.motorEfficiency >= 0.5
                        diagnosis{it}.status{4} = 'prop ok';
                    else
                        diagnosis{it}.status{4} = 'prop loss';
                    end
                end 
            else
                diagnosis = [];
            end
        end  
        function positionControlOutput = positionControl(obj, desiredState)
        %UNTITLED Summary of this function goes here
        %   Detailed explanation goes here
            if ~obj.isRunning()
                obj.positionControlConfig_.ierror = zeros(3,1);
            end    
            error = desiredState.position-obj.previousState_.position;           %Position error
            obj.positionControlConfig_.ierror = obj.positionControlConfig_.ierror+error*obj.controlTimeStep_;
            derror = desiredState.velocity-obj.previousState_.velocity;          %Derivative of the error    
            dderror = desiredState.acceleration-obj.previousState_.acceleration; %Second derivative of the error
            positionControlOutput = obj.positionControlConfig_.kp'.*error+obj.positionControlConfig_.ki'.*obj.positionControlConfig_.ierror+obj.positionControlConfig_.kd'.*derror + obj.positionControlConfig_.kdd'.*dderror + obj.mass_*[0;0;9.81];
        end 
        function desiredAttitude = attitudeReference(obj, desiredImpulse, desiredYaw, diagnosis)
        %UNTITLED Summary of this function goes here
        %   Detailed explanation goes here
            % Desired Attitude Calculation

            % Step 1 - Find desired force vector in the body frame at the desired
            % yaw direction
            yawd = desiredYaw;
            qyd = [cos(yawd/2) 0 0 sin(yawd/2)];
            Qyd = obj.matrixBtoA(qyd);
            invQyd = Qyd';
            Tcd = invQyd*desiredImpulse;
            switch obj.attReferenceAlg_
                case 1 % PI Passive
                    % Calculate aircraft inverse control model
                    Mf = [];
                    for it=1:obj.numberOfRotors_
                        Mf = [Mf obj.rotorLiftCoeff(it,obj.rotorOperatingPoint_(it))*obj.rotor_(it).orientation];
                    end
                    % Step 2 - Calculates rotor speeds that generate that force vector,
                    % forcing the aircraft to stay at hover
                    invMf = pinv(Mf);
                    [omega_square] = invMf*Tcd;
                    % Step 3 - If any rotor speed is less than zero, assumes it is zero
                    omega_square(omega_square<0) = 0;
                case 2 % PI Active
                    % Calculate aircraft inverse control model
                    Mf = [];
                    for it=1:obj.numberOfRotors_
                        if strcmp('stuck',diagnosis{it}.status{1})
                            Mf = [Mf [0 0 0]'];
                        else
                            Mf = [Mf (obj.rotorLiftCoeff(it,obj.rotorOperatingPoint_(it))*obj.rotor_(it).orientation)*(diagnosis{it}.propEfficiency*diagnosis{it}.motorEfficiency^2)];
                        end
                    end
                    % Step 2 - Calculates rotor speeds that generate that force vector,
                    % forcing the aircraft to stay at hover
                    invMf = pinv(Mf);
                    [omega_square] = invMf*Tcd;
                    % Step 3 - If any rotor speed is less than zero, assumes it is zero
                    omega_square(omega_square<0) = 0;
                case 3 % RPI Passive
                    tau = Tcd;
                    Mf = [];
                    for it=1:obj.numberOfRotors_
                        Mf = [Mf obj.rotorLiftCoeff(it,obj.rotorOperatingPoint_(it))*obj.rotor_(it).orientation];
                    end
                    auxMf = Mf;
                    c = zeros(obj.numberOfRotors_,1);
                    inLoop = true;
                    maxSpeeds = ((obj.rotorMaxSpeed((1:obj.numberOfRotors_))).^2)';
                    minSpeeds = ((obj.rotorMinSpeed((1:obj.numberOfRotors_))).^2)';
                    while inLoop
                        [omega_square] = -c + pinv(auxMf)*(tau+Mf*c);
                        omega_square(abs(omega_square)<1) = 0;
                        if all(omega_square<=maxSpeeds) && all(omega_square>=minSpeeds)
                            inLoop = false;
                        else
                            for it=1:obj.numberOfRotors_
                                if omega_square(it)<=minSpeeds(it)
                                    c(it) = -minSpeeds(it);
                                    auxMf(:,it) = zeros(size(auxMf(:,it)));
                                elseif omega_square(it)>=maxSpeeds(it)
                                    c(it) = -maxSpeeds(it);
                                    auxMf(:,it) = zeros(size(auxMf(:,it)));   
                                end
                            end
                        end
                    end
                case 4 % RPI Active
                    tau = Tcd;
                    Mf = [];
                    for it=1:obj.numberOfRotors_
                        if strcmp('stuck',diagnosis{it}.status{1})
                            Mf = [Mf [0 0 0]'];
                        else
                            Mf = [Mf (obj.rotorLiftCoeff(it,obj.rotorOperatingPoint_(it))*obj.rotor_(it).orientation)*(diagnosis{it}.propEfficiency*diagnosis{it}.motorEfficiency^2)];
                        end
                    end
                    auxMf = Mf;
                    c = zeros(obj.numberOfRotors_,1);
                    inLoop = true;
                    for it=1:obj.numberOfRotors_
                        if strcmp('stuck',diagnosis{it}.status{1})
                            maxSpeeds(it,:) = 0;
                            minSpeeds(it,:) = 0;
                        else
                            maxSpeeds(it,:) = (obj.rotorMaxSpeed(it)*diagnosis{it}.motorEfficiency)^2;
                            minSpeeds(it,:) = (obj.rotorMinSpeed(it)*diagnosis{it}.motorEfficiency)^2;
                        end
                    end
                    while inLoop
                        [omega_square] = -c + pinv(auxMf)*(tau+Mf*c);
                        omega_square(abs(omega_square)<1) = 0;
                        if all(omega_square<=maxSpeeds) && all(omega_square>=minSpeeds)
                            inLoop = false;
                        else
                            for it=1:obj.numberOfRotors_
                                if omega_square(it)<=minSpeeds(it)
                                    c(it) = -minSpeeds(it);
                                    auxMf(:,it) = zeros(size(auxMf(:,it)));
                                elseif omega_square(it)>=maxSpeeds(it)
                                    c(it) = -maxSpeeds(it);
                                    auxMf(:,it) = zeros(size(auxMf(:,it)));   
                                end
                            end
                        end
                    end
                case 5 % Adaptive
                    index = 5;
                    if ~obj.isRunning()
                        Mf = [];
                        Mt = [];
                        torqueAux = zeros(3,1);
                        for it=1:obj.numberOfRotors_
                            Mf = [Mf (obj.rotorLiftCoeff(it,obj.rotorOperatingPoint_(it))*obj.rotor_(it).orientation)];
                            Mt = [Mt (obj.rotorLiftCoeff(it,obj.rotorOperatingPoint_(it))*cross(obj.rotor_(it).position,obj.rotor_(it).orientation)-obj.rotorDragCoeff(it,obj.rotorOperatingPoint_(it))*obj.rotorDirection_(it)*obj.rotor_(it).orientation)];
                            torqueAux = torqueAux + obj.rotorOrientation(it)*(obj.previousRotorSpeed(it).*obj.rotorInertia(it));
                        end                    
                        obj.allocationConfig_{index}.Bpest = [Mf/obj.mass();obj.inertia()\Mt];
                    end
                    Bpest = obj.allocationConfig_{index}.Bpest;
                    Mf = obj.mass()*Bpest(1:3,:);
                    Mt = obj.inertia()*Bpest(4:6,:);
                    maxSpeeds = ((obj.rotorMaxSpeed(1:obj.numberOfRotors_)).^2)';
                    minSpeeds = ((obj.rotorMinSpeed(1:obj.numberOfRotors_)).^2)';
                    op = (maxSpeeds+minSpeeds)/2; % in the middle of the squared rotor speed range, to best maneuverability
                    N = null(Mt);
                    R = obj.allocationConfig_{index}.R;
                    Q = obj.allocationConfig_{index}.Q;
                    v = (N'*(R*N+Mf'*Q*Mf*N))\(N'*(R*op+Mf'*Q*Tcd));
                    omega_square = N*v;
                    lessIndex = omega_square<minSpeeds;
                    greaterIndex = omega_square>maxSpeeds;
                    omega_square(lessIndex) = minSpeeds(lessIndex);
                    omega_square(greaterIndex) = maxSpeeds(greaterIndex);
                case 7 % Passive NMAC
                    index = 7;
                    if ~obj.isRunning()
                        obj.allocationConfig_{index}.Mf = [];
                        obj.allocationConfig_{index}.Mt = [];
                        for it=1:obj.numberOfRotors_
                            obj.allocationConfig_{index}.Mf = [obj.allocationConfig_{index}.Mf obj.rotorLiftCoeff(it,obj.rotorOperatingPoint_(it))*obj.rotor_(it).orientation];
                            obj.allocationConfig_{index}.Mt = [obj.allocationConfig_{index}.Mt (obj.rotorLiftCoeff(it,obj.rotorOperatingPoint_(it))*cross(obj.rotor_(it).position,obj.rotor_(it).orientation)-obj.rotorDragCoeff(it,obj.rotorOperatingPoint_(it))*obj.rotorDirection_(it)*obj.rotor_(it).orientation)];
                        end
                        obj.allocationConfig_{index}.maxSpeeds = ((obj.rotorMaxSpeed(1:obj.numberOfRotors_)).^2)';
                        obj.allocationConfig_{index}.minSpeeds = ((obj.rotorMinSpeed(1:obj.numberOfRotors_)).^2)';
                        obj.allocationConfig_{index}.op = (obj.allocationConfig_{index}.maxSpeeds+obj.allocationConfig_{index}.minSpeeds)/2; % in the middle of the squared rotor speed range, to best maneuverability
                        obj.allocationConfig_{index}.N = null(obj.allocationConfig_{index}.Mt);
                        
                    end
                    Mf = obj.allocationConfig_{index}.Mf;
                    maxSpeeds = obj.allocationConfig_{index}.maxSpeeds;
                    minSpeeds = obj.allocationConfig_{index}.minSpeeds;
                    op = obj.allocationConfig_{index}.op;
                    N = obj.allocationConfig_{index}.N;
                    R = obj.allocationConfig_{index}.R;
                    Q = obj.allocationConfig_{index}.Q;
                    v = (N'*(R*N+Mf'*Q*Mf*N))\(N'*(R*op+Mf'*Q*Tcd));
                    omega_square = N*v;
                    lessIndex = omega_square<minSpeeds;
                    greaterIndex = omega_square>maxSpeeds;
                    omega_square(lessIndex) = minSpeeds(lessIndex);
                    omega_square(greaterIndex) = maxSpeeds(greaterIndex);
                case 8 % Active NMAC
                    index = 8;
                    Mf = [];
                    Mt = [];
                    for it=1:obj.numberOfRotors_
                        if strcmp('stuck',diagnosis{it}.status{1})
                            Mf = [Mf [0 0 0]'];
                            Mt = [Mt [0 0 0]'];
                        else
                            Mf = [Mf (obj.rotorLiftCoeff(it,obj.rotorOperatingPoint_(it))*obj.rotor_(it).orientation)*(diagnosis{it}.propEfficiency*diagnosis{it}.motorEfficiency^2)];
                            Mt = [Mt (obj.rotorLiftCoeff(it,obj.rotorOperatingPoint_(it))*cross(obj.rotor_(it).position,obj.rotor_(it).orientation)-obj.rotorDragCoeff(it,obj.rotorOperatingPoint_(it))*obj.rotorDirection_(it)*obj.rotor_(it).orientation)*(diagnosis{it}.propEfficiency*diagnosis{it}.motorEfficiency^2)];
                        end
                    end
                    maxSpeeds = zeros(obj.numberOfRotors_,1);
                    minSpeeds = zeros(obj.numberOfRotors_,1);
                    for it=1:obj.numberOfRotors_
                        if strcmp('stuck',diagnosis{it}.status{1})
                            maxSpeeds(it,:) = 0;
                            minSpeeds(it,:) = 0;
                        else
                            maxSpeeds(it,:) = (obj.rotorMaxSpeed(it)*diagnosis{it}.motorEfficiency)^2;
                            minSpeeds(it,:) = (obj.rotorMinSpeed(it)*diagnosis{it}.motorEfficiency)^2;
                        end
                    end
                    op = (maxSpeeds+minSpeeds)/2; % in the middle of the squared rotor speed range, to best maneuverability
                    N = null(Mt);
                    R = obj.allocationConfig_{index}.R;
                    Q = obj.allocationConfig_{index}.Q;
                    v = (N'*(R*N+Mf'*Q*Mf*N))\(N'*(R*op+Mf'*Q*Tcd));
                    omega_square = N*v;
                    lessIndex = omega_square<minSpeeds;
                    greaterIndex = omega_square>maxSpeeds;
                    omega_square(lessIndex) = minSpeeds(lessIndex);
                    omega_square(greaterIndex) = maxSpeeds(greaterIndex);
                otherwise
                    error('No allocation method available')
            end              
            % Step 4 - Find the force vector that the aircraft is capable of
            % generating
            Tc = Mf*omega_square;
            % Step 5 - Find the attitude to align the force vector generated by
            % aircraft to the desired force vector
            if desiredImpulse(3)<0
                desiredImpulse(3) = - desiredImpulse(3);
            end
            Ta = Qyd*Tc;
            if norm(Ta)==0
                thetaCB = acos(([0 0 1]*desiredImpulse)/(norm(desiredImpulse)));
                vCB = cross(([0 0 1]'),(desiredImpulse/norm(desiredImpulse)));
            else
                thetaCB = acos((Ta'*desiredImpulse)/(norm(Ta)*norm(desiredImpulse)));
                vCB = cross((Ta/norm(Ta)),(desiredImpulse/norm(desiredImpulse)));
            end
            thetaCB = real(thetaCB);
            %thetaDegress = thetaCB*180/pi
            qCB = [cos(thetaCB/2) vCB'*sin(thetaCB/2)];
            % compound rotation -> desired attitude
            desiredAttitude(1) = qCB(1)*qyd(1)-qCB(2)*qyd(2)-qCB(3)*qyd(3)-qCB(4)*qyd(4);
            desiredAttitude(2) = qCB(1)*qyd(2)+qCB(2)*qyd(1)+qCB(3)*qyd(4)-qCB(4)*qyd(3);
            desiredAttitude(3) = qCB(1)*qyd(3)-qCB(2)*qyd(4)+qCB(3)*qyd(1)+qCB(4)*qyd(2);
            desiredAttitude(4) = qCB(1)*qyd(4)+qCB(2)*qyd(3)-qCB(3)*qyd(2)+qCB(4)*qyd(1);
            
            desiredAttitude = desiredAttitude/norm(desiredAttitude);
        end  
        function attitudeControlOutput = control(obj, desiredAttitude, desiredImpulse,diagnosis)
        %UNTITLED Summary of this function goes here
        %   Detailed explanation goes here
            filterGain = obj.filterConfig_.angular;
            if ~obj.isRunning()
                obj.velocityFilter_.Wbe = [0;0;0];
                obj.velocityFilter_.angularVelocity = [0;0;0];
                obj.velocityFilter_.desiredAngularVelocity = [0;0;0];
            end

            previousWbe = obj.velocityFilter_.Wbe;
            previousAngularVelocity = obj.velocityFilter_.angularVelocity;
            angularVelocity = obj.previousAngularVelocity();
            obj.velocityFilter_.angularVelocity = angularVelocity;            
            
            qd = desiredAttitude;
            q = obj.previousState_.attitude; %Current attitude
            % Quaternion error2
            qe(1) = + qd(1)*q(1) + qd(2)*q(2) + qd(3)*q(3) + qd(4)*q(4);
            qe(2) = + qd(2)*q(1) - qd(1)*q(2) - qd(4)*q(3) + qd(3)*q(4);
            qe(3) = + qd(3)*q(1) + qd(4)*q(2) - qd(1)*q(3) - qd(2)*q(4);
            qe(4) = + qd(4)*q(1) - qd(3)*q(2) + qd(2)*q(3) - qd(1)*q(4);
            if(qe(1)<0)
                qe(2:4) = -qe(2:4);
            end 
            
            obj.velocityFilter_.desiredAngularVelocity = filterGain*obj.velocityFilter_.desiredAngularVelocity+(eye(3)-filterGain)*(qe(2:4)'/obj.controlTimeStep_);
            desiredAngularVelocity = obj.velocityFilter_.desiredAngularVelocity;   
            obj.trajectory_.angularVelocity(:,end+1) = desiredAngularVelocity;
            angularAcceleration = (angularVelocity-previousAngularVelocity)/obj.controlTimeStep_;
            Wbe = desiredAngularVelocity-angularVelocity;
            obj.velocityFilter_.Wbe = Wbe;
            dWbe = (Wbe-previousWbe)/obj.controlTimeStep_;
%             desiredAngularAcceleration = (desiredAngularVelocity - obj.previousState_.angularVelocity)/obj.controlTimeStep_
%             desiredAngularVelocity
            desiredAngularAcceleration = (dWbe+angularAcceleration);
%             desiredAngularAcceleration = [0;0;0];
%             desiredAngularVelocity = [0;0;0];
%             desiredAngularAcceleration = 2*(desiredAngularVelocity-angularVelocity)/(obj.controlTimeStep_)
            
            switch obj.controlAlg_
                case 1 %'PID'
                    if ~obj.isRunning()
                        obj.controlConfig_{1}.ierror = zeros(3,1);
                    end    
                    errorVector = [qe(2); qe(3); qe(4)];
                    obj.controlConfig_{1}.ierror = obj.controlConfig_{1}.ierror+errorVector*obj.controlTimeStep_;
                    attitudeControlOutput = obj.inertiaTensor_*(-reshape(obj.controlConfig_{1}.kd,[3 1]).*angularVelocity+reshape(obj.controlConfig_{1}.kp,[3 1]).*errorVector+reshape(obj.controlConfig_{1}.ki,[3 1]).*obj.controlConfig_{1}.ierror);
                case 2 %'RLQ-R Passive'
                    index = 2;
                    
                    rotorIDs = 1:obj.numberOfRotors();
                    torqueAux = [obj.rotor_(rotorIDs).orientation]*([obj.previousState_.rotor(rotorIDs).speed].*obj.rotorInertia(rotorIDs))';
                    auxA = obj.inertia()*angularVelocity-torqueAux;
                    auxA = [0 -auxA(3) auxA(2) ; auxA(3) 0 -auxA(1) ; -auxA(2) auxA(1) 0 ];
                    auxA = obj.inertia()\auxA;

                    Sq = [qe(1),-qe(4),qe(3);
                          qe(4),qe(1),-qe(2);
                          -qe(3),qe(2),qe(1)];
                    
                    x_e = [Wbe;qe(2:4)'];

                    ssA = [auxA, zeros(3); 0.5*Sq, zeros(3)];
                    ssB = [eye(3);zeros(3)];
                    sys = ss(ssA,ssB,eye(6),0);
                    sysD = c2d(sys,obj.controlTimeStep_);

                    F = sysD.A;
                    G = sysD.B;
                    if ~obj.isRunning()
                        obj.controlConfig_{index}.P = obj.controlConfig_{index}.initialP;
                    end
                    P = obj.controlConfig_{index}.P;
                    R = obj.controlConfig_{index}.R;
                    Q = obj.controlConfig_{index}.Q;
                    Ef = obj.controlConfig_{index}.Ef;
                    Eg = obj.controlConfig_{index}.Eg;
                    H = obj.controlConfig_{index}.H;
                    mu = obj.controlConfig_{index}.mu;
                    alpha = obj.controlConfig_{index}.alpha;

                    [~,K,P] = obj.gainRLQR(P,R,Q,mu,alpha,F,G,H,Ef,Eg);
                    obj.controlConfig_{index}.P = P;
                    u = K*x_e;
                    attitudeControlOutput = -obj.inertia()*(u-desiredAngularAcceleration+auxA*desiredAngularVelocity);
                    %attitudeControlOutput = obj.inertia()*(u+desiredAngularAcceleration-auxA*desiredAngularVelocity);
                case 3 %'RLQ-R Passive Modified'
                    index = 3;                    
                    Mt = [];
                    torqueAux = zeros(3,1);
                    for it=1:obj.numberOfRotors_
                        Mt = [Mt (obj.rotorLiftCoeff(it,obj.rotorOperatingPoint_(it))*cross(obj.rotor_(it).position,obj.rotor_(it).orientation)-obj.rotorDragCoeff(it,obj.rotorOperatingPoint_(it))*obj.rotorDirection_(it)*obj.rotor_(it).orientation)];
                        torqueAux = torqueAux + obj.rotorOrientation(it)*(obj.previousRotorSpeed(it)*obj.rotorInertia(it));
                    end
                    
                    auxA = obj.inertia()*obj.previousAngularVelocity()-torqueAux;
                    auxA = [0 -auxA(3) auxA(2) ; auxA(3) 0 -auxA(1) ; -auxA(2) auxA(1) 0 ];
                    auxA = obj.inertia()\auxA;

                    Sq = [qe(1),-qe(4),qe(3);
                          qe(4),qe(1),-qe(2);
                          -qe(3),qe(2),qe(1)];

                    x_e = [desiredAngularVelocity-obj.previousAngularVelocity();qe(2:4)'];
                    
                    C = -obj.inertia()\Mt;
                    ssA = [auxA, zeros(3); 0.5*Sq, zeros(3)];
                    ssB = [C;zeros(3,size(C,2))];
                    sys = ss(ssA,ssB,eye(6),0);
                    sysD = c2d(sys,obj.controlTimeStep_);

                    F = sysD.A;
                    G = sysD.B;
                    if ~obj.isRunning()
                        obj.controlConfig_{index}.P = obj.controlConfig_{index}.initialP;
                    end
                    P = obj.controlConfig_{index}.P;
                    R = obj.controlConfig_{index}.R;
                    Q = obj.controlConfig_{index}.Q;
                    Ef = obj.controlConfig_{index}.Ef;
                    Eg = obj.controlConfig_{index}.Eg;
                    H = obj.controlConfig_{index}.H;
                    mu = obj.controlConfig_{index}.mu;
                    alpha = obj.controlConfig_{index}.alpha;

                    [~,K,P] = obj.gainRLQR(P,R,Q,mu,alpha,F,G,H,Ef,Eg);
                    obj.controlConfig_{index}.P = P;
                    u = K*x_e;
                    attitudeControlOutput = -obj.inertia()*(C*u-desiredAngularAcceleration+auxA*desiredAngularVelocity);  
                case 4 %'RLQ-R Passive Modified with PIDD' 
                    index = 4;
                    
                    invQ = obj.matrixAbsoluteToBody();
                    Q = invQ';
                    Abd = invQ*desiredImpulse/obj.mass();
                    %Abd = invQ*(desiredImpulse+obj.mass_*[0;0;-9.81])/obj.mass();
                    Vb = invQ*obj.previousVelocity();
                    Vbd = Abd*obj.controlTimeStep_ + Vb;
                    dVbd = Abd;
                    Ve = Vbd-Vb;
                    
                    Mf = [];
                    Mt = [];
                    torqueAux = zeros(3,1);
                    for it=1:obj.numberOfRotors_
                        Mf = [Mf obj.rotorLiftCoeff(it,obj.rotorOperatingPoint_(it))*obj.rotor_(it).orientation];
                        Mt = [Mt (obj.rotorLiftCoeff(it,obj.rotorOperatingPoint_(it))*cross(obj.rotor_(it).position,obj.rotor_(it).orientation)-obj.rotorDragCoeff(it,obj.rotorOperatingPoint_(it))*obj.rotorDirection_(it)*obj.rotor_(it).orientation)];
                        torqueAux = torqueAux + obj.rotorOrientation(it)*(obj.previousRotorSpeed(it)*obj.rotorInertia(it));
                    end
                    
                    auxA = obj.inertia()*obj.previousAngularVelocity()-torqueAux;
                    auxA = [0 -auxA(3) auxA(2) ; auxA(3) 0 -auxA(1) ; -auxA(2) auxA(1) 0 ];
                    auxA = obj.inertia()\auxA;

                    Sq = [qe(1),-qe(4),qe(3);
                          qe(4),qe(1),-qe(2);
                          -qe(3),qe(2),qe(1)];

                    x_e = [Ve;desiredAngularVelocity-obj.previousAngularVelocity();qe(2:4)'];
                    
                    C = -obj.inertia()\Mt;
                    ssA = [-obj.friction()*eye(3)/obj.mass(), zeros(3,6); zeros(3), auxA, zeros(3); zeros(3), 0.5*Sq, zeros(3)];
                    ssB = [Mf/obj.mass();C;zeros(3,size(C,2))];
                    sys = ss(ssA,ssB,eye(9),0);
                    sysD = c2d(sys,obj.controlTimeStep_);

                    F = sysD.A;
                    G = sysD.B;
                    if ~obj.isRunning()
                        obj.controlConfig_{index}.P = obj.controlConfig_{index}.initialP;
                    end
                    P = obj.controlConfig_{index}.P;
                    R = obj.controlConfig_{index}.R;
                    Q = obj.controlConfig_{index}.Q;
                    Ef = obj.controlConfig_{index}.Ef;
                    Eg = obj.controlConfig_{index}.Eg;
                    H = obj.controlConfig_{index}.H;
                    mu = obj.controlConfig_{index}.mu;
                    alpha = obj.controlConfig_{index}.alpha;

                    [~,K,P] = obj.gainRLQR(P,R,Q,mu,alpha,F,G,H,Ef,Eg);
                    obj.controlConfig_{index}.P = P;
                    u = K*x_e;
                    attitudeControlOutput.torque = -obj.inertia()*(C*u-desiredAngularAcceleration+auxA*desiredAngularVelocity);
                    attitudeControlOutput.impulse = obj.mass()*dVbd-Mf*u+obj.friction()*Vbd+obj.mass()*invQ*[0;0;-9.81]; % Impulse in relation to body frame
                case 5 %'RLQ-R Active'
                    index = 5;
                    
                    rotorIDs = 1:obj.numberOfRotors();
                    propEfficiency = [];
                    motorEfficiency = [];
                    for it=1:obj.numberOfRotors_
                        propEfficiency = [propEfficiency, diagnosis{it}.propEfficiency];
                        motorEfficiency = [motorEfficiency, diagnosis{it}.motorEfficiency];
                    end
                    torqueAux = obj.rotorOrientation(rotorIDs)*(obj.previousRotorSpeed(rotorIDs).*obj.rotorInertia(rotorIDs).*motorEfficiency.*(propEfficiency.^2))';
                    auxA = obj.inertia()*obj.previousAngularVelocity()-torqueAux;
                    auxA = [0 -auxA(3) auxA(2) ; auxA(3) 0 -auxA(1) ; -auxA(2) auxA(1) 0 ];
                    auxA = obj.inertia()\auxA;

                    Sq = [qe(1),-qe(4),qe(3);
                          qe(4),qe(1),-qe(2);
                          -qe(3),qe(2),qe(1)];

                    x_e = [Wbe;qe(2:4)'];

                    ssA = [auxA, zeros(3); 0.5*Sq, zeros(3)];
                    ssB = [eye(3);zeros(3)];
                    sys = ss(ssA,ssB,eye(6),0);
                    sysD = c2d(sys,obj.controlTimeStep_);

                    F = sysD.A;
                    G = sysD.B;
                    if ~obj.isRunning()
                        obj.controlConfig_{index}.P = obj.controlConfig_{index}.initialP;
                    end
                    P = obj.controlConfig_{index}.P;
                    R = obj.controlConfig_{index}.R;
                    Q = obj.controlConfig_{index}.Q;
                    Ef = obj.controlConfig_{index}.Ef;
                    Eg = obj.controlConfig_{index}.Eg;
                    H = obj.controlConfig_{index}.H;
                    mu = obj.controlConfig_{index}.mu;
                    alpha = obj.controlConfig_{index}.alpha;

                    [~,K,P] = obj.gainRLQR(P,R,Q,mu,alpha,F,G,H,Ef,Eg);
                    obj.controlConfig_{index}.P = P;
                    u = K*x_e;
                    attitudeControlOutput = -obj.inertia()*(u-desiredAngularAcceleration+auxA*desiredAngularVelocity);
                    %attitudeControlOutput = obj.inertia()*(u+desiredAngularAcceleration-auxA*desiredAngularVelocity);
                case 6 %'RLQ-R Active Modified'
                    index = 6;
                    diagnosis{1}.motorEfficiency
                    Mt = [];
                    torqueAux = zeros(3,1);
                    for it=1:obj.numberOfRotors_
                        if strcmp('stuck',diagnosis{it}.status{1})
                            Mt = [Mt [0 0 0]'];
                        else
                            Mt = [Mt (obj.rotorLiftCoeff(it,obj.rotorOperatingPoint_(it))*cross(obj.rotor_(it).position,obj.rotor_(it).orientation)-obj.rotorDragCoeff(it,obj.rotorOperatingPoint_(it))*obj.rotorDirection_(it)*obj.rotor_(it).orientation)*(diagnosis{it}.propEfficiency*diagnosis{it}.motorEfficiency^2)];
                        end
                        torqueAux = torqueAux + obj.rotorOrientation(it)*(obj.previousRotorSpeed(it)*obj.rotorInertia(it)*diagnosis{it}.motorEfficiency*(diagnosis{it}.propEfficiency^2));
                    end
                    
                    auxA = obj.inertia()*obj.previousAngularVelocity()-torqueAux;
                    auxA = [0 -auxA(3) auxA(2) ; auxA(3) 0 -auxA(1) ; -auxA(2) auxA(1) 0 ];
                    auxA = obj.inertia()\auxA;

                    Sq = [qe(1),-qe(4),qe(3);
                          qe(4),qe(1),-qe(2);
                          -qe(3),qe(2),qe(1)];

                    x_e = [desiredAngularVelocity-obj.previousAngularVelocity();qe(2:4)'];
                                      
                    C = -obj.inertia()\Mt;
                    ssA = [auxA, zeros(3); 0.5*Sq, zeros(3)];
                    ssB = [C;zeros(3,size(C,2))];
                    sys = ss(ssA,ssB,eye(6),0);
                    sysD = c2d(sys,obj.controlTimeStep_);

                    F = sysD.A;
                    G = sysD.B;
                    if ~obj.isRunning()
                        obj.controlConfig_{index}.P = obj.controlConfig_{index}.initialP;
                    end
                    P = obj.controlConfig_{index}.P;
                    R = obj.controlConfig_{index}.R;
                    Q = obj.controlConfig_{index}.Q;
                    Ef = obj.controlConfig_{index}.Ef;
                    Eg = obj.controlConfig_{index}.Eg;
                    H = obj.controlConfig_{index}.H;
                    mu = obj.controlConfig_{index}.mu;
                    alpha = obj.controlConfig_{index}.alpha;

                    [~,K,P] = obj.gainRLQR(P,R,Q,mu,alpha,F,G,H,Ef,Eg);
                    obj.controlConfig_{index}.P = P;
                    u = K*x_e;
                    attitudeControlOutput = -obj.inertia()*(C*u-desiredAngularAcceleration+auxA*desiredAngularVelocity); 
                case 7 %'RLQ-R Active Modified with PIDD'
                    index = 7;
                    
                    invQ = obj.matrixAbsoluteToBody();
                    Q = invQ';
                    Abd = invQ*desiredImpulse/obj.mass();
                    Vb = invQ*obj.previousVelocity();
                    Vbd = Abd*obj.controlTimeStep_ + Vb;
                    dVbd = Abd;
                    Ve = Vbd-Vb;
                    
                    Mf = [];
                    Mt = [];
                    torqueAux = zeros(3,1);
                    for it=1:obj.numberOfRotors_
                        if strcmp('stuck',diagnosis{it}.status{1})
                            Mf = [Mf [0 0 0]'];
                            Mt = [Mt [0 0 0]'];
                        else
                            Mf = [Mf (obj.rotorLiftCoeff(it,obj.rotorOperatingPoint_(it))*obj.rotor_(it).orientation)*(diagnosis{it}.propEfficiency*diagnosis{it}.motorEfficiency^2)];
                            Mt = [Mt (obj.rotorLiftCoeff(it,obj.rotorOperatingPoint_(it))*cross(obj.rotor_(it).position,obj.rotor_(it).orientation)-obj.rotorDragCoeff(it,obj.rotorOperatingPoint_(it))*obj.rotorDirection_(it)*obj.rotor_(it).orientation)*(diagnosis{it}.propEfficiency*diagnosis{it}.motorEfficiency^2)];
                        end
                        torqueAux = torqueAux + obj.rotorOrientation(it)*(obj.previousRotorSpeed(it).*obj.rotorInertia(it)*diagnosis{it}.motorEfficiency*(diagnosis{it}.propEfficiency^2));
                    end  
                    
                    auxA = obj.inertia()*obj.previousAngularVelocity()-torqueAux;
                    auxA = [0 -auxA(3) auxA(2) ; auxA(3) 0 -auxA(1) ; -auxA(2) auxA(1) 0 ];
                    auxA = obj.inertia()\auxA;

                    Sq = [qe(1),-qe(4),qe(3);
                          qe(4),qe(1),-qe(2);
                          -qe(3),qe(2),qe(1)];

                    x_e = [Ve;desiredAngularVelocity-obj.previousAngularVelocity();qe(2:4)'];
                    
                    C = -obj.inertia()\Mt;
                    ssA = [-obj.friction()*eye(3)/obj.mass(), zeros(3,6); zeros(3), auxA, zeros(3); zeros(3), 0.5*Sq, zeros(3)];
                    ssB = [Mf/obj.mass();C;zeros(3,size(C,2))];
                    sys = ss(ssA,ssB,eye(9),0);
                    sysD = c2d(sys,obj.controlTimeStep_);

                    F = sysD.A;
                    G = sysD.B;
                    if ~obj.isRunning()
                        obj.controlConfig_{index}.P = obj.controlConfig_{index}.initialP;
                    end
                    P = obj.controlConfig_{index}.P;
                    R = obj.controlConfig_{index}.R;
                    Q = obj.controlConfig_{index}.Q;
                    Ef = obj.controlConfig_{index}.Ef;
                    Eg = obj.controlConfig_{index}.Eg;
                    H = obj.controlConfig_{index}.H;
                    mu = obj.controlConfig_{index}.mu;
                    alpha = obj.controlConfig_{index}.alpha;

                    [~,K,P] = obj.gainRLQR(P,R,Q,mu,alpha,F,G,H,Ef,Eg);
                    obj.controlConfig_{index}.P = P;
                    u = K*x_e;
                    attitudeControlOutput.torque = -obj.inertia()*(C*u-desiredAngularAcceleration+auxA*desiredAngularVelocity);
                    attitudeControlOutput.impulse = obj.mass()*dVbd-Mf*u+obj.friction()*Vbd+obj.mass()*invQ*[0;0;-9.81]; % Impulse in relation to body frame
                case 8 %'SOSMC Passive'
                    index = 8;
                    rotorIDs = 1:obj.numberOfRotors();
                    c = obj.controlConfig_{index}.c;
                    lambda = obj.controlConfig_{index}.lambda;
                    alpha = obj.controlConfig_{index}.alpha;
                    if ~obj.isRunning()
                        obj.controlConfig_{index}.integral = zeros(3,1);
                    end
                    obj.controlConfig_{index}.error = qe(2:4)';
                    e = qe(2:4)';
                    torqueAux = obj.rotorOrientation(rotorIDs)*(obj.previousRotorSpeed(rotorIDs).*obj.rotorInertia(rotorIDs))';
                    auxA = obj.inertia()*obj.previousAngularVelocity()-torqueAux;
                    auxA = cross(auxA,obj.previousAngularVelocity());
                    fx = obj.inertia()\auxA;
                    
                    s = Wbe+c*e;
                    signS = min(max(s, -1), 1);
                    obj.controlConfig_{index}.integral = obj.controlConfig_{index}.integral + obj.controlTimeStep_*alpha*signS; %%%%% importante
                    ueq = obj.inertia()*(desiredAngularAcceleration+c*Wbe-fx);
                    udis = +lambda*(signS.*(s.^2))+obj.controlConfig_{index}.integral;
                    
                    attitudeControlOutput = ueq+udis;                   
                case 9 %'SOSMC Passive with PIDD' 
                    index = 9;
                    c = obj.controlConfig_{index}.c;
                    lambda = obj.controlConfig_{index}.lambda;
                    alpha = obj.controlConfig_{index}.alpha;
                    if ~obj.isRunning()
                        obj.controlConfig_{index}.integral = zeros(6,1);
                        obj.controlConfig_{index}.error = zeros(6,1);
                        obj.controlConfig_{index}.position = obj.previousState_.position;
                        %obj.debug = [];
                    end
                    
                    invQ = obj.matrixAbsoluteToBody();
                    Tad = desiredImpulse+obj.mass_*[0;0;-9.81];
                    desiredAcceleration = invQ*(Tad/obj.mass_);
                    
                    previousPosition = obj.controlConfig_{index}.position;
                    obj.controlConfig_{index}.position = obj.previousState_.position;
                    currentPosition = obj.previousState_.position;
                    
                    obj.controlConfig_{index}.error = [qe(2:4)';invQ*(Tad*(obj.controlTimeStep_^2)/obj.mass_+currentPosition-previousPosition)];
                    %obj.controlConfig_{index}.error = [qe(2:4)';invQ*(desiredState.position-obj.previousState_.position)];
                    e = obj.controlConfig_{index}.error;
                    de = [desiredAngularVelocity - obj.previousState_.angularVelocity;desiredAcceleration*obj.controlTimeStep_];
                    %de = (e-previousError)/obj.controlTimeStep_;
                    
                    torqueAux = zeros(3,1);
                    for it=1:obj.numberOfRotors_
                        torqueAux = torqueAux + obj.rotorOrientation(it)*(obj.previousRotorSpeed(it)*obj.rotorInertia(it));
                    end
                    
                    auxA = obj.inertia()*obj.previousAngularVelocity()-torqueAux;                    
                    auxA = cross(auxA,obj.previousAngularVelocity());
                    fx = [obj.inertia()\auxA; invQ*[0;0;-9.81]-obj.friction()*invQ*obj.previousState_.velocity/obj.mass_];
                    
                    gx = [obj.inertia()\eye(3),zeros(3);zeros(3),eye(3)/obj.mass_];
                    invgx = gx^(-1);
                    ddxd = [desiredAngularAcceleration;desiredAcceleration];
                    
                    s = de+c*e;
                    signS = min(max(s, -1), 1);
                    obj.controlConfig_{index}.integral = obj.controlConfig_{index}.integral + obj.controlTimeStep_*alpha*signS; %%%%% importante
                    ueq = invgx*(ddxd+c*de-fx);
                    udis = +lambda*(signS.*(s.^2))+ obj.controlConfig_{index}.integral;
                    
                    u=ueq+udis;
                    attitudeControlOutput.torque = u(1:3);
                    attitudeControlOutput.impulse = u(4:6); % Impulse in relation to body frame
                case 10 %'SOSMC Passive Direct'
                    index = 10;
                    c = obj.controlConfig_{index}.c;
                    lambda = obj.controlConfig_{index}.lambda;
                    alpha = obj.controlConfig_{index}.alpha;
                    if ~obj.isRunning()
                        obj.controlConfig_{index}.integral = zeros(obj.numberOfRotors_,1);
                        obj.controlConfig_{index}.error = zeros(6,1);
                        obj.controlConfig_{index}.position = obj.previousState_.position;
                        %obj.debug = [];
                    end
                    
                    invQ = obj.matrixAbsoluteToBody();
                    Tad = desiredImpulse+obj.mass_*[0;0;-9.81];
                    desiredAcceleration = invQ*(Tad/obj.mass_);
                    
                    previousPosition = obj.controlConfig_{index}.position;
                    obj.controlConfig_{index}.position = obj.previousState_.position;
                    currentPosition = obj.previousState_.position;
                    
                    obj.controlConfig_{index}.error = [qe(2:4)';invQ*(Tad*(obj.controlTimeStep_^2)/obj.mass_+currentPosition-previousPosition)];
                    %obj.controlConfig_{index}.error = [qe(2:4)';invQ*(desiredState.position-obj.previousState_.position)];
                    e = obj.controlConfig_{index}.error;
                    de = [desiredAngularVelocity - obj.previousState_.angularVelocity;desiredAcceleration*obj.controlTimeStep_];
                    %de = (e-previousError)/obj.controlTimeStep_;
                    
                    Mf = [];
                    Mt = [];
                    torqueAux = zeros(3,1);
                    for it=1:obj.numberOfRotors_
                        Mf = [Mf obj.rotorLiftCoeff(it,obj.rotorOperatingPoint_(it))*obj.rotor_(it).orientation];
                        Mt = [Mt (obj.rotorLiftCoeff(it,obj.rotorOperatingPoint_(it))*cross(obj.rotor_(it).position,obj.rotor_(it).orientation)-obj.rotorDragCoeff(it,obj.rotorOperatingPoint_(it))*obj.rotorDirection_(it)*obj.rotor_(it).orientation)];
                        torqueAux = torqueAux + obj.rotorOrientation(it)*(obj.previousRotorSpeed(it)*obj.rotorInertia(it));
                    end
                    
                    auxA = obj.inertia()*obj.previousAngularVelocity()-torqueAux;
                    auxA = cross(auxA,obj.previousAngularVelocity());
                    fx = [obj.inertia()\auxA; invQ*[0;0;-9.81]-obj.friction()*invQ*obj.previousState_.velocity/obj.mass_];
                    
                    %gx = [obj.inertia()\Mt;Mf/obj.mass_];
                    %invgx = pinv(gx);
                    ddxd = [desiredAngularAcceleration;desiredAcceleration];
                    
                    s = de+c*e;
                    signS = min(max(s, -1), 1);                    
                    obj.controlConfig_{index}.integral = obj.controlConfig_{index}.integral + obj.controlTimeStep_*alpha*signS; %%%%% importante
                    
                    maxSpeeds = ((obj.rotorMaxSpeed(1:obj.numberOfRotors_)).^2)';
                    minSpeeds = ((obj.rotorMinSpeed(1:obj.numberOfRotors_)).^2)';
                    op = (maxSpeeds+minSpeeds)/2; % in the middle of the squared rotor speed range, to best maneuverability
                    N = null(Mt);
                    R = obj.allocationConfig_{index}.R;
                    Q = obj.allocationConfig_{index}.Q;
                    accelerations = (ddxd+c*de-fx);
                    desiredImpulse = accelerations(4:6)*obj.mass();
                    desiredTorque = obj.inertia()*accelerations(1:3);
                    v = (N'*(R*N+Mf'*Q*Mf*N))\(N'*(R*op+Mf'*Q*desiredImpulse));
                    b2 = N*v;
                    utau = pinv(Mt)*desiredTorque;
                    c = (desiredImpulse-Mf*utau)'*Mf*b2/(norm(Mf*b2)^2);
                    ueq = utau+b2*c;
                    
                    %ueq = invgx*(ddxd+c*de-fx);
                    udis = +lambda*(signS.*(s.^2))+obj.controlConfig_{index}.integral;
                    
                    [omega_square]=ueq+udis;
                    omega_square(omega_square<0) = 0;
                    for it=1:obj.numberOfRotors_
                        attitudeControlOutput(it,1) = obj.rotorDirection_(it)*sqrt(omega_square(it));
                    end 
                    %realizedImpulse = obj.matrixBodyToAbsolute()*Mf*omega_square
                    %Mf*omega_square
                    %obj.debug = [obj.debug [e;de;s;obj.controlConfig_{index}.integral;ueq;udis;attitudeControlOutput]];
                    %pause
                case 11 %'SOSMC Active'
                    index = 11;
                    rotorIDs = 1:obj.numberOfRotors();
                    c = obj.controlConfig_{index}.c;
                    lambda = obj.controlConfig_{index}.lambda;
                    alpha = obj.controlConfig_{index}.alpha;
                    if ~obj.isRunning()
                        obj.controlConfig_{index}.integral = zeros(3,1);
                    end
                    
                    obj.controlConfig_{index}.error = qe(2:4)';
                    e = qe(2:4)';                    
                    propEfficiency = [];
                    motorEfficiency = [];
                    for it=1:obj.numberOfRotors_
                        propEfficiency = [propEfficiency, diagnosis{it}.propEfficiency];
                        motorEfficiency = [motorEfficiency, diagnosis{it}.motorEfficiency];
                    end
                    torqueAux = obj.rotorOrientation(rotorIDs)*(obj.previousRotorSpeed(rotorIDs).*obj.rotorInertia(rotorIDs).*motorEfficiency.*(propEfficiency.^2))';                    
                    auxA = obj.inertia()*obj.previousAngularVelocity()-torqueAux;
                    auxA = cross(auxA,obj.previousAngularVelocity());
                    fx = obj.inertia()\auxA;
                    
                    s = Wbe+c*e;
                    signS = min(max(s, -1), 1);  
                    obj.controlConfig_{index}.integral = obj.controlConfig_{index}.integral + obj.controlTimeStep_*alpha*signS; %%%%% importante
                    ueq = obj.inertia()*(desiredAngularAcceleration+c*Wbe-fx);
                    udis = lambda*(signS.*(s.^2)) + obj.controlConfig_{index}.integral;
                                       
                    attitudeControlOutput = ueq+udis;
                case 12 %'SOSMC Active with PIDD'
                    index = 12;
                    c = obj.controlConfig_{index}.c;
                    lambda = obj.controlConfig_{index}.lambda;
                    alpha = obj.controlConfig_{index}.alpha;
                    if ~obj.isRunning()
                        obj.controlConfig_{index}.integral = zeros(6,1);
                        obj.controlConfig_{index}.error = zeros(6,1);
                        obj.controlConfig_{index}.position = obj.previousState_.position;
                        %obj.debug = [];
                    end
                    
                    invQ = obj.matrixAbsoluteToBody();
                    Tad = desiredImpulse+obj.mass_*[0;0;-9.81];
                    desiredAcceleration = invQ*(Tad/obj.mass_);
                    
                    previousPosition = obj.controlConfig_{index}.position;
                    obj.controlConfig_{index}.position = obj.previousState_.position;
                    currentPosition = obj.previousState_.position;
                    
                    obj.controlConfig_{index}.error = [qe(2:4)';invQ*(Tad*(obj.controlTimeStep_^2)/obj.mass_+currentPosition-previousPosition)];
                    %obj.controlConfig_{index}.error = [qe(2:4)';invQ*(desiredState.position-obj.previousState_.position)];
                    e = obj.controlConfig_{index}.error;
                    de = [desiredAngularVelocity - obj.previousState_.angularVelocity;desiredAcceleration*obj.controlTimeStep_];
                    %de = (e-previousError)/obj.controlTimeStep_;
                    
                    torqueAux = zeros(3,1);
                    for it=1:obj.numberOfRotors_
                        torqueAux = torqueAux + obj.rotorOrientation(it)*(obj.previousRotorSpeed(it)*obj.rotorInertia(it))*(diagnosis{it}.propEfficiency^2)*diagnosis{it}.motorEfficiency;
                    end
                    
                    auxA = obj.inertia()*obj.previousAngularVelocity()-torqueAux;
                    auxA = cross(auxA,obj.previousAngularVelocity());
                    fx = [obj.inertia()\auxA; invQ*[0;0;-9.81]-obj.friction()*invQ*obj.previousState_.velocity/obj.mass_];
                    
                    gx = [obj.inertia()\eye(3),zeros(3);zeros(3),eye(3)/obj.mass_];
                    invgx = gx^(-1);
                    ddxd = [desiredAngularAcceleration;desiredAcceleration];
                    
                    s = de+c*e;
                    signS = min(max(s, -1), 1);  
                    obj.controlConfig_{index}.integral = obj.controlConfig_{index}.integral + obj.controlTimeStep_*alpha*signS; %%%%% importante
                    ueq = invgx*(ddxd+c*de-fx);
                    udis = -lambda*(signS.*(s.^2))- obj.controlConfig_{index}.integral;
                    
                    u=ueq-udis;
                    attitudeControlOutput.torque = u(1:3);
                    attitudeControlOutput.impulse = u(4:6); % Impulse in relation to body frame
                case 13 %'SOSMC Active Direct'
                    index = 13;
                    c = obj.controlConfig_{index}.c;
                    lambda = obj.controlConfig_{index}.lambda;
                    alpha = obj.controlConfig_{index}.alpha;
                    if ~obj.isRunning()
                        obj.controlConfig_{index}.integral = zeros(obj.numberOfRotors_,1);
                        obj.controlConfig_{index}.error = zeros(6,1);
                        obj.controlConfig_{index}.position = obj.previousState_.position;
                        %obj.debug = [];
                    end
                    
                    invQ = obj.matrixAbsoluteToBody();
                    Tad = desiredImpulse+obj.mass_*[0;0;-9.81];
                    desiredAcceleration = invQ*(Tad/obj.mass_);
                    
                    previousPosition = obj.controlConfig_{index}.position;
                    obj.controlConfig_{index}.position = obj.previousState_.position;
                    currentPosition = obj.previousState_.position;
                    
                    obj.controlConfig_{index}.error = [qe(2:4)';invQ*(Tad*(obj.controlTimeStep_^2)/obj.mass_+currentPosition-previousPosition)];
                    %obj.controlConfig_{index}.error = [qe(2:4)';invQ*(desiredState.position-obj.previousState_.position)];
                    e = obj.controlConfig_{index}.error;
                    de = [desiredAngularVelocity - obj.previousState_.angularVelocity;desiredAcceleration*obj.controlTimeStep_];
                    %de = (e-previousError)/obj.controlTimeStep_;
                    
                    Mf = [];
                    Mt = [];
                    torqueAux = zeros(3,1);
                    for it=1:obj.numberOfRotors_
                        if strcmp('stuck',diagnosis{it}.status{1})
                            Mf = [Mf [0 0 0]'];
                            Mt = [Mt [0 0 0]'];
                        else
                            Mf = [Mf (obj.rotorLiftCoeff(it,obj.rotorOperatingPoint_(it))*obj.rotor_(it).orientation)*(diagnosis{it}.propEfficiency*diagnosis{it}.motorEfficiency^2)];
                            Mt = [Mt (obj.rotorLiftCoeff(it,obj.rotorOperatingPoint_(it))*cross(obj.rotor_(it).position,obj.rotor_(it).orientation)-obj.rotorDragCoeff(it,obj.rotorOperatingPoint_(it))*obj.rotorDirection_(it)*obj.rotor_(it).orientation)*(diagnosis{it}.propEfficiency*diagnosis{it}.motorEfficiency^2)];
                        end
                        torqueAux = torqueAux + obj.rotorOrientation(it)*(obj.previousRotorSpeed(it).*obj.rotorInertia(it)*diagnosis{it}.motorEfficiency*(diagnosis{it}.propEfficiency^2));
                    end                    
                    auxA = obj.inertia()*obj.previousAngularVelocity()-torqueAux;
                    auxA = cross(auxA,obj.previousAngularVelocity());
                    fx = [obj.inertia()\auxA; invQ*[0;0;-9.81]-obj.friction()*invQ*obj.previousState_.velocity/obj.mass_];
                    
                    %gx = [obj.inertia()\Mt;Mf/obj.mass_];
                    %invgx = pinv(gx);
                    ddxd = [desiredAngularAcceleration;desiredAcceleration];
                    
                    s = de+c*e;
                    signS = min(max(s, -1), 1);  
                    obj.controlConfig_{index}.integral = obj.controlConfig_{index}.integral + obj.controlTimeStep_*alpha*signS; %%%%% importante
                    
                    maxSpeeds = zeros(obj.numberOfRotors_,1);
                    minSpeeds = zeros(obj.numberOfRotors_,1);
                    for it=1:obj.numberOfRotors_
                        if strcmp('stuck',diagnosis{it}.status{1})
                            maxSpeeds(it,:) = 0;
                            minSpeeds(it,:) = 0;
                        else
                            maxSpeeds(it,:) = (obj.rotorMaxSpeed(it)*diagnosis{it}.motorEfficiency)^2;
                            minSpeeds(it,:) = (obj.rotorMinSpeed(it)*diagnosis{it}.motorEfficiency)^2;
                        end
                    end
                    op = (maxSpeeds+minSpeeds)/2; % in the middle of the squared rotor speed range, to best maneuverability
                    N = null(Mt);
                    R = obj.allocationConfig_{index}.R;
                    Q = obj.allocationConfig_{index}.Q;
                    accelerations = (ddxd+c*de-fx);
                    desiredImpulse = accelerations(4:6)*obj.mass();
                    desiredTorque = obj.inertia()*accelerations(1:3);
                    v = (N'*(R*N+Mf'*Q*Mf*N))\(N'*(R*op+Mf'*Q*desiredImpulse));
                    b2 = N*v;
                    utau = pinv(Mt)*desiredTorque;
                    c = (desiredImpulse-Mf*utau)'*Mf*b2/(norm(Mf*b2)^2);
                    ueq = utau+b2*c;
                    
                    %ueq = invgx*(ddxd+c*de-fx);
                    udis = +lambda*(signS.*(s.^2))+obj.controlConfig_{index}.integral;
                    
                    [omega_square]=ueq+udis;
                    omega_square(omega_square<0) = 0;
                    for it=1:obj.numberOfRotors_
                        attitudeControlOutput(it,1) = obj.rotorDirection_(it)*sqrt(omega_square(it));
                    end 
                    %realizedImpulse = obj.matrixBodyToAbsolute()*Mf*omega_square
                    %Mf*omega_square
                    %obj.debug = [obj.debug [e;de;s;obj.controlConfig_{index}.integral;ueq;udis;attitudeControlOutput]];
                    %pause
                case 14 %'Adaptive' 
                    index = 14;
                    Am = obj.controlConfig_{index}.Am;
                    Bm = eye(3);
                    Bp = obj.inertia()\eye(3);
                    if ~obj.isRunning()
                        obj.controlConfig_{index}.P = lyap(obj.controlConfig_{index}.Am',obj.controlConfig_{index}.Q);
                        % Discretiza��o do erro delta
                        sysEdelta = ss(Am,Bp,eye(3),0);
                        sysEdeltaD = c2d(sysEdelta,obj.controlTimeStep_);
                        obj.controlConfig_{index}.AmEd = sysEdeltaD.A;
                        obj.controlConfig_{index}.BpEd = sysEdeltaD.B;
                        % Discretiza��o do estado desejado Xm
                        sysXm = ss(Am,Bm,eye(3),0);
                        sysXmD = c2d(sysXm,obj.controlTimeStep_);
                        obj.controlConfig_{index}.AmXd = sysXmD.A;
                        obj.controlConfig_{index}.BmXd = sysXmD.B;
                        
                        % Vari�veis auxiliares
                        obj.controlConfig_{index}.eDelta = zeros(3,1);
                        obj.controlConfig_{index}.xm = zeros(3,1);
                        obj.controlConfig_{index}.r = zeros(3,1);
                        obj.controlConfig_{index}.x = zeros(3,1);
                        obj.controlConfig_{index}.lambda = zeros(3,1);
                        obj.controlConfig_{index}.f = zeros(3,1);
                        obj.controlConfig_{index}.Kr = obj.inertia()*Bm;
                        obj.controlConfig_{index}.Kx = obj.inertia()*Am;
                        obj.controlConfig_{index}.u = zeros(3,1);
                    end
                    % Updates auxiliary variables
                    previouseDelta = obj.controlConfig_{index}.eDelta;
                    previousxm = obj.controlConfig_{index}.xm;
                    previouslambda = obj.controlConfig_{index}.lambda;
                    previousf = obj.controlConfig_{index}.f;
                    previousKr = obj.controlConfig_{index}.Kr;
                    previousKx = obj.controlConfig_{index}.Kx;
                    previousu = obj.controlConfig_{index}.u;
                    r = desiredAngularAcceleration;
                    previousx = obj.controlConfig_{index}.x;
                    x = obj.previousState_.angularVelocity;  
                    obj.controlConfig_{index}.x = x;
                    
                    obj.controlConfig_{index}.Mt = [];
                    for it=1:obj.numberOfRotors_
                        %obj.controlConfig_{index}.Mt = [obj.controlConfig_{index}.Mt (obj.rotorLiftCoeff(it,obj.rotorOperatingPoint_(it))*cross(obj.rotor_(it).position,obj.rotor_(it).orientation)-obj.rotorDragCoeff(it,obj.rotorOperatingPoint_(it))*obj.rotorDirection_(it)*obj.rotor_(it).orientation)];
                        obj.controlConfig_{index}.Mt = [obj.controlConfig_{index}.Mt (obj.rotorLiftCoeff(it,obj.previousState_.rotor(it).speed)*cross(obj.rotor_(it).position,obj.rotor_(it).orientation)-obj.rotorDragCoeff(it,obj.previousState_.rotor(it).speed)*obj.rotorDirection_(it)*obj.rotor_(it).orientation)];
                    end
                    
                    % Calculates current reference state
                    obj.controlConfig_{index}.xm = obj.controlConfig_{index}.AmXd*previousxm+obj.controlConfig_{index}.BmXd*r;
                    
                    % Calculates current error
                    % deltaU
                    previousSatInput = [obj.previousState_.rotor(:).speed]';
                    for i=1:obj.numberOfRotors_
                        if abs(previousSatInput(i))>obj.rotor_(i).maxSpeed
                            previousSatInput(i) = obj.rotor_(i).maxSpeed*sign(previousSatInput(i));
                        end
                        if abs(previousSatInput(i))<obj.rotor_(i).minSpeed
                            previousSatInput(i) = obj.rotor_(i).minSpeed*sign(previousSatInput(i));
                        end
                    end     
                    previousSatU = obj.controlConfig_{index}.Mt*(previousSatInput.^2);
                    deltaU = previousSatU - previousu;
                    % eDelta
                    obj.controlConfig_{index}.eDelta = obj.controlConfig_{index}.AmEd*previouseDelta+obj.controlConfig_{index}.BpEd*diag(previouslambda)*deltaU;
                    % state error
                    e = x-previousxm;
                    % estimate error
                    eu = e - obj.controlConfig_{index}.eDelta;
%                     eu = e;
                    
                    % Update gains
                    obj.controlConfig_{index}.Kx = -obj.controlTimeStep_*obj.controlConfig_{index}.gamma1*Bp'*obj.controlConfig_{index}.P*eu*x'+previousKx;
%                     Kx = obj.controlConfig_{index}.Kx
                    obj.controlConfig_{index}.Kr = -obj.controlTimeStep_*obj.controlConfig_{index}.gamma2*Bp'*obj.controlConfig_{index}.P*eu*r'+previousKr;
%                     Kr = obj.controlConfig_{index}.Kr
                    obj.controlConfig_{index}.f = -obj.controlTimeStep_*obj.controlConfig_{index}.gamma3*Bp'*obj.controlConfig_{index}.P*eu+previousf;
                    obj.controlConfig_{index}.lambda = obj.controlTimeStep_*obj.controlConfig_{index}.gamma4*diag(deltaU)*Bp'*obj.controlConfig_{index}.P*eu+previouslambda;
                    % Calculates control output
                    attitudeControlOutput = previousKx*x+previousKr*r+previousf;
                    obj.controlConfig_{index}.u = attitudeControlOutput;
%                     u = attitudeControlOutput
                    
%                     index = 14;
%                     Am = obj.controlConfig_{index}.Am;
%                     Bp = [obj.inertia()\eye(3);zeros(4,3)];
%                     if ~obj.isRunning()
%                         % Discretiza��o do erro delta
%                         sysEdelta = ss(Am,Bp,eye(7),0);
%                         sysEdeltaD = c2d(sysEdelta,obj.controlTimeStep_);
%                         obj.controlConfig_{index}.AmEd = sysEdeltaD.A;
%                         obj.controlConfig_{index}.BpEd = sysEdeltaD.B;
%                         
%                         obj.controlConfig_{index}.P = lyap(Am,obj.controlConfig_{index}.Q);
%                         % Vari�veis auxiliares
%                         qm = obj.previousState_.attitude;
%                         Sqm = [-qm(2),-qm(3),-qm(4);
%                               qm(1),-qm(4),qm(3);
%                               qm(4),qm(1),-qm(2);
%                               -qm(3),qm(2),qm(1)];
%                         Bm = [eye(3),zeros(3);zeros(4,3),0.5*Sqm];
%                         obj.controlConfig_{index}.eDelta = zeros(7,1);
%                         obj.controlConfig_{index}.xm = [obj.previousState_.angularVelocity;obj.previousState_.attitude];
%                         obj.controlConfig_{index}.r = zeros(3,1);
%                         obj.controlConfig_{index}.x = [obj.previousState_.angularVelocity;obj.previousState_.attitude];
%                         obj.controlConfig_{index}.lambda = ones(3,1);
%                         obj.controlConfig_{index}.f = zeros(3,1);
%                         obj.controlConfig_{index}.Kr = pinv(Bp)*Bm;
%                         obj.controlConfig_{index}.Kx = pinv(Bp)*(Am-[zeros(4,7);0.5*eye(3),zeros(3,4)]);
%                         obj.controlConfig_{index}.u = zeros(3,1);
%                     end
%                     % Updates auxiliary variables
%                     previouseDelta = obj.controlConfig_{index}.eDelta;
%                     previousxm = obj.controlConfig_{index}.xm;
%                     previouslambda = obj.controlConfig_{index}.lambda;
%                     previousf = obj.controlConfig_{index}.f;
%                     previousKr = obj.controlConfig_{index}.Kr;
%                     previousKx = obj.controlConfig_{index}.Kx;
%                     previousu = obj.controlConfig_{index}.u;
%                     r = [desiredAngularAcceleration;desiredAngularVelocity];
%                     previousx = obj.controlConfig_{index}.x;
%                     x = [obj.previousState_.angularVelocity;obj.previousState_.attitude];  
%                     obj.controlConfig_{index}.x = x;
%                     
%                     qm = previousxm(4:7)/norm(previousxm(4:7));
%                     Sqm = [-qm(2),-qm(3),-qm(4);
%                           qm(1),-qm(4),qm(3);
%                           qm(4),qm(1),-qm(2);
%                           -qm(3),qm(2),qm(1)];
%                     Bm = [eye(3),zeros(3);zeros(4,3),0.5*Sqm];
%                     
%                     % Discretiza��o do estado desejado Xm
%                     sysXm = ss(Am,Bm,eye(7),0);
%                     sysXmD = c2d(sysXm,obj.controlTimeStep_);
%                     obj.controlConfig_{index}.AmXd = sysXmD.A;
%                     obj.controlConfig_{index}.BmXd = sysXmD.B;
%                     
%                     obj.controlConfig_{index}.Mt = [];
%                     for it=1:obj.numberOfRotors_
%                         obj.controlConfig_{index}.Mt = [obj.controlConfig_{index}.Mt (obj.rotorLiftCoeff(it,obj.rotorOperatingPoint_(it))*cross(obj.rotor_(it).position,obj.rotor_(it).orientation)-obj.rotorDragCoeff(it,obj.rotorOperatingPoint_(it))*obj.rotorDirection_(it)*obj.rotor_(it).orientation)];
%                     end
%                     
%                     % Calculates current reference state
%                     obj.controlConfig_{index}.xm = obj.controlConfig_{index}.AmXd*previousxm+obj.controlConfig_{index}.BmXd*r;
%                     
%                     % Calculates current error
%                     % deltaU
%                     previousSatInput = obj.previousInput();
%                     for i=1:obj.numberOfRotors_
%                         if abs(previousSatInput(i))>obj.rotor_(i).maxSpeed
%                             previousSatInput(i) = obj.rotor_(i).maxSpeed*sign(previousSatInput(i));
%                         end
%                         if abs(previousSatInput(i))<obj.rotor_(i).minSpeed
%                             previousSatInput(i) = obj.rotor_(i).minSpeed*sign(previousSatInput(i));
%                         end
%                     end     
%                     previousSatU = obj.controlConfig_{index}.Mt*(previousSatInput.^2);
%                     deltaU = previousSatU - previousu;
%                     % eDelta
%                     obj.controlConfig_{index}.eDelta = obj.controlConfig_{index}.AmEd*previouseDelta+obj.controlConfig_{index}.BpEd*diag(previouslambda)*deltaU;
%                     % state error
% %                     previousxm
%                     x
%                     previousxm
%                     e = x-previousxm;
%                     % estimate error
% %                     eu = e - obj.controlConfig_{index}.eDelta
%                     eu = e;
%                     
%                     % Update gains
%                     obj.controlConfig_{index}.Kx = -obj.controlTimeStep_*obj.controlConfig_{index}.gamma1*Bp'*obj.controlConfig_{index}.P*eu*x'+previousKx;
% %                     Kx = obj.controlConfig_{index}.Kx
%                     obj.controlConfig_{index}.Kr = -obj.controlTimeStep_*obj.controlConfig_{index}.gamma2*Bp'*obj.controlConfig_{index}.P*eu*r'+previousKr;
% %                     Kr = obj.controlConfig_{index}.Kr
%                     obj.controlConfig_{index}.f = -obj.controlTimeStep_*obj.controlConfig_{index}.gamma3*Bp'*obj.controlConfig_{index}.P*eu+previousf;
%                     obj.controlConfig_{index}.lambda = obj.controlTimeStep_*obj.controlConfig_{index}.gamma4*diag(deltaU)*Bp'*obj.controlConfig_{index}.P*eu+previouslambda;
%                     % Calculates control output
%                     attitudeControlOutput = previousKx*x+previousKr*r+previousf;
%                     obj.controlConfig_{index}.u = attitudeControlOutput;
                case 15 %'Adaptive with PIDD'
                    index = 15;
                    invQ = obj.matrixAbsoluteToBody();
                    r = [invQ*desiredImpulse;desiredAngularAcceleration];
                    x = [obj.previousVelocity();obj.previousAngularVelocity()];    
                    Am = obj.controlConfig_{index}.Am;
                    Bm = [eye(3)/obj.mass(),zeros(3,3);zeros(3,3),eye(3)];
                    if ~obj.isRunning()
                        obj.controlConfig_{index}.P = lyap(obj.controlConfig_{index}.Am',obj.controlConfig_{index}.Q);                        
                        % C�lculo Bp  
                        Bp = [eye(3)/obj.mass_,zeros(3);zeros(3),obj.inertia()\eye(3)];
                        obj.controlConfig_{index}.Bp = Bp;
                        % Discretiza��o do erro delta
                        sysEdelta = ss(Am,Bp,eye(6),0);
                        sysEdeltaD = c2d(sysEdelta,obj.controlTimeStep_);
                        obj.controlConfig_{index}.AmEd = sysEdeltaD.A;
                        obj.controlConfig_{index}.BpEd = sysEdeltaD.B;
                        % Discretiza��o do estado desejado Xm
                        sysXm = ss(Am,Bm,eye(6),0);
                        sysXmD = c2d(sysXm,obj.controlTimeStep_);
                        obj.controlConfig_{index}.AmXd = sysXmD.A;
                        obj.controlConfig_{index}.BmXd = sysXmD.B;
                        
                        % Vari�veis auxiliares
                        obj.controlConfig_{index}.eDelta = zeros(6,1);
                        obj.controlConfig_{index}.zm = [0;0;1;0;0;0];
                        obj.controlConfig_{index}.lambda = zeros(6,1);
                        obj.controlConfig_{index}.f = zeros(6,1);
                        obj.controlConfig_{index}.Kr = pinv(Bp)*Bm;
                        obj.controlConfig_{index}.Kx = pinv(Bp)*Am;
                        obj.controlConfig_{index}.u = zeros(6,1);
                    end                    
                    
                    % Updates auxiliary variables
                    previouseDelta = obj.controlConfig_{index}.eDelta;
                    previouszm = obj.controlConfig_{index}.zm;
                    previouslambda = obj.controlConfig_{index}.lambda;
                    previousf = obj.controlConfig_{index}.f;
                    previousKr = obj.controlConfig_{index}.Kr;
                    previousKx = obj.controlConfig_{index}.Kx;
                    previousu = obj.controlConfig_{index}.u;
                    Bp = obj.controlConfig_{index}.Bp;
                    Mf = zeros(3,obj.numberOfRotors_);
                    Mt = zeros(3,obj.numberOfRotors_);
                    for it=1:obj.numberOfRotors_
                        %Mf(:,it) = obj.rotorLiftCoeff(it,obj.rotorOperatingPoint_(it))*obj.rotor_(it).orientation;
                        %Mt(:,it) = (obj.rotorLiftCoeff(it,obj.rotorOperatingPoint_(it))*cross(obj.rotor_(it).position,obj.rotor_(it).orientation)-obj.rotorDragCoeff(it,obj.rotorOperatingPoint_(it))*obj.rotorDirection_(it)*obj.rotor_(it).orientation);
                        Mf(:,it) = obj.rotorLiftCoeff(it,obj.previousState_.rotor(it).speed)*obj.rotor_(it).orientation;
                        Mt(:,it) = (obj.rotorLiftCoeff(it,obj.previousState_.rotor(it).speed)*cross(obj.rotor_(it).position,obj.rotor_(it).orientation)-obj.rotorDragCoeff(it,obj.previousState_.rotor(it).speed)*obj.rotorDirection_(it)*obj.rotor_(it).orientation);
                    end
                    obj.controlConfig_{index}.Mf = Mf;
                    obj.controlConfig_{index}.Mt = Mt;
                    
                    % Calculates current reference state
                    obj.controlConfig_{index}.zm = obj.controlConfig_{index}.AmXd*previouszm+obj.controlConfig_{index}.BmXd*r;
                    
                    % Calculates current error
                    % deltaU
                    previousSatInput = [obj.previousState_.rotor(:).speed]';
                    for i=1:obj.numberOfRotors_
                        if abs(previousSatInput(i))>obj.rotor_(i).maxSpeed
                            previousSatInput(i) = obj.rotor_(i).maxSpeed*sign(previousSatInput(i));
                        end
                        if abs(previousSatInput(i))<obj.rotor_(i).minSpeed
                            previousSatInput(i) = obj.rotor_(i).minSpeed*sign(previousSatInput(i));
                        end
                    end     
                    previousSatU = [obj.controlConfig_{index}.Mf;obj.controlConfig_{index}.Mt]*(previousSatInput.^2);
                    deltaU = previousSatU - previousu;
                    % eDelta
                    obj.controlConfig_{index}.eDelta = obj.controlConfig_{index}.AmEd*previouseDelta+obj.controlConfig_{index}.BpEd*diag(previouslambda)*deltaU;
                    % state error
                    e = x-(previouszm-Am\[invQ*[0;0;-9.81];zeros(3,1)]);
                    % estimate error
                    eu = e - obj.controlConfig_{index}.eDelta;
                    
                    % Update gains
                    obj.controlConfig_{index}.Kx = -obj.controlTimeStep_*obj.controlConfig_{index}.gamma1*Bp'*obj.controlConfig_{index}.P*eu*x'+previousKx;
%                     Kx = obj.controlConfig_{index}.Kx
                    obj.controlConfig_{index}.Kr = -obj.controlTimeStep_*obj.controlConfig_{index}.gamma2*Bp'*obj.controlConfig_{index}.P*eu*r'+previousKr;
%                     Kr = obj.controlConfig_{index}.Kr
                    obj.controlConfig_{index}.f = -obj.controlTimeStep_*obj.controlConfig_{index}.gamma3*Bp'*obj.controlConfig_{index}.P*eu+previousf;
                    obj.controlConfig_{index}.lambda = obj.controlTimeStep_*obj.controlConfig_{index}.gamma4*diag(deltaU)*Bp'*obj.controlConfig_{index}.P*eu+previouslambda;
%                     lambda = previouslambda
%                     f = obj.controlConfig_{index}.f
                    % Calculates control output
                    obj.controlConfig_{index}.u = previousKx*x+previousKr*r+previousf;
                    attitudeControlOutput.impulse = obj.controlConfig_{index}.u(1:3);
                    attitudeControlOutput.torque = obj.controlConfig_{index}.u(4:6); % Impulse in relation to body frame
                case 16 %'Adaptive Direct'
                    index = 16;
                    invQ = obj.matrixAbsoluteToBody();
                    r = [invQ*desiredImpulse;desiredAngularAcceleration];
                    x = [obj.previousVelocity();obj.previousAngularVelocity];    
                    Am = obj.controlConfig_{index}.Am;
                    Bm = [eye(3)/obj.mass(),zeros(3,3);zeros(3,3),eye(3)];
                    if ~obj.isRunning()
                        obj.controlConfig_{index}.P = lyap(obj.controlConfig_{index}.Am',obj.controlConfig_{index}.Q);                        
                        % C�lculo Bp
                        Mf = zeros(3,obj.numberOfRotors_);
                        Mt = zeros(3,obj.numberOfRotors_);
                        for it=1:obj.numberOfRotors_
                            Mf(:,it) = obj.rotorLiftCoeff(it,obj.previousState_.rotor(it).speed)*obj.rotor_(it).orientation;
                            Mt(:,it) = (obj.rotorLiftCoeff(it,obj.previousState_.rotor(it).speed)*cross(obj.rotor_(it).position,obj.rotor_(it).orientation)-obj.rotorDragCoeff(it,obj.previousState_.rotor(it).speed)*obj.rotorDirection_(it)*obj.rotor_(it).orientation);
                        end
                        Bp = [Mf/obj.mass();obj.inertia()\Mt];
                        obj.controlConfig_{index}.Bp = Bp;
                        % Discretiza��o do erro delta
                        sysEdelta = ss(Am,Bp,eye(6),0);
                        sysEdeltaD = c2d(sysEdelta,obj.controlTimeStep_);
                        obj.controlConfig_{index}.AmEd = sysEdeltaD.A;
                        obj.controlConfig_{index}.BpEd = sysEdeltaD.B;
                        % Discretiza��o do estado desejado Xm
                        sysXm = ss(Am,Bm,eye(6),0);
                        sysXmD = c2d(sysXm,obj.controlTimeStep_);
                        obj.controlConfig_{index}.AmXd = sysXmD.A;
                        obj.controlConfig_{index}.BmXd = sysXmD.B;
                        
                        % Vari�veis auxiliares
                        obj.controlConfig_{index}.eDelta = zeros(6,1);
                        obj.controlConfig_{index}.zm = [0;0;1;0;0;0];
                        obj.controlConfig_{index}.lambda = ones(obj.numberOfRotors(),1);
                        obj.controlConfig_{index}.f = zeros(obj.numberOfRotors(),1);
                        
                        %obj.controlConfig_{index}.Kr = pinv(Bp)*Bm;
                        %obj.controlConfig_{index}.Kx = pinv(Bp)*Am;
                        obj.controlConfig_{index}.Kr = obj.controlConfig_{index}.B0*Bm;
                        obj.controlConfig_{index}.Kx = obj.controlConfig_{index}.B0*Am;
                        obj.controlConfig_{index}.u = zeros(obj.numberOfRotors(),1);
                    end                    
                    
                    % Updates auxiliary variables
                    previouseDelta = obj.controlConfig_{index}.eDelta;
                    previouszm = obj.controlConfig_{index}.zm;
                    previouslambda = obj.controlConfig_{index}.lambda;
                    previousf = obj.controlConfig_{index}.f;
                    previousKr = obj.controlConfig_{index}.Kr;
                    previousKx = obj.controlConfig_{index}.Kx;
                    previousu = obj.controlConfig_{index}.u;
                    Mf = zeros(3,obj.numberOfRotors_);
                    Mt = zeros(3,obj.numberOfRotors_);
                    for it=1:obj.numberOfRotors_
                        Mf(:,it) = obj.rotorLiftCoeff(it,obj.previousState_.rotor(it).speed)*obj.rotor_(it).orientation;
                        Mt(:,it) = (obj.rotorLiftCoeff(it,obj.previousState_.rotor(it).speed)*cross(obj.rotor_(it).position,obj.rotor_(it).orientation)-obj.rotorDragCoeff(it,obj.previousState_.rotor(it).speed)*obj.rotorDirection_(it)*obj.rotor_(it).orientation);
                    end
                    Bp = [Mf/obj.mass();obj.inertia()\Mt];
                    obj.controlConfig_{index}.Bp = Bp;
                    
                    % Calculates current reference state
                    obj.controlConfig_{index}.zm = obj.controlConfig_{index}.AmXd*previouszm+obj.controlConfig_{index}.BmXd*r;
                    
                    % Calculates current error
                    % deltaU
                    previousSatInput = [obj.previousState_.rotor(:).speed]';
                    for i=1:obj.numberOfRotors_
                        if abs(previousSatInput(i))>obj.rotor_(i).maxSpeed
                            previousSatInput(i) = obj.rotor_(i).maxSpeed*sign(previousSatInput(i));
                        end
                        if abs(previousSatInput(i))<obj.rotor_(i).minSpeed
                            previousSatInput(i) = obj.rotor_(i).minSpeed*sign(previousSatInput(i));
                        end
                    end     
                    previousSatU = previousSatInput.^2;
                    deltaU = previousSatU - previousu;
                    % eDelta
                    obj.controlConfig_{index}.eDelta = obj.controlConfig_{index}.AmEd*previouseDelta+obj.controlConfig_{index}.BpEd*diag(previouslambda)*deltaU;
                    % state error
                    e = x-(previouszm-Am\[invQ*[0;0;-9.81];zeros(3,1)]);
                    % estimate error
                    eu = e - obj.controlConfig_{index}.eDelta;
                    
                    % Update gains
                    obj.controlConfig_{index}.Kx = -obj.controlTimeStep_*obj.controlConfig_{index}.gamma1*Bp'*obj.controlConfig_{index}.P*eu*x'+previousKx;
                    obj.controlConfig_{index}.Kr = -obj.controlTimeStep_*obj.controlConfig_{index}.gamma2*Bp'*obj.controlConfig_{index}.P*eu*r'+previousKr;
                    obj.controlConfig_{index}.f = -obj.controlTimeStep_*obj.controlConfig_{index}.gamma3*Bp'*obj.controlConfig_{index}.P*eu+previousf;
                    obj.controlConfig_{index}.lambda = obj.controlTimeStep_*obj.controlConfig_{index}.gamma4*diag(deltaU)*Bp'*obj.controlConfig_{index}.P*eu+previouslambda;
%                     lambda = previouslambda
%                     f = obj.controlConfig_{index}.f
                    % Calculates control output
                    obj.controlConfig_{index}.u = previousKx*x+previousKr*r+previousf;
                    omega_square = obj.controlConfig_{index}.u;
                    omega_square(omega_square<0) = 0;
                    for it=1:obj.numberOfRotors_
                        attitudeControlOutput(it,1) = obj.rotorDirection_(it)*sqrt(omega_square(it));
                    end
                    
%                     Kx = obj.controlConfig_{index}.Kx
%                     Kr = obj.controlConfig_{index}.Kr
%                     f = obj.controlConfig_{index}.f
%                     lambda = obj.controlConfig_{index}.lambda
%                     desiredTorque = obj.inertia()*desiredAngularAcceleration
%                     executedTorque = Bp*omega_square;
%                     executedTorque = obj.inertia()*executedTorque(4:6)
                case 17 %'Markovian RLQ-R Passive Modified'
                    index = 17; 
                    
                    if ~obj.isRunning()
                        obj.controlConfig_{index}.P = obj.controlConfig_{index}.initialP;
                        obj.controlConfig_{index}.Ee = [];
                        Ef = obj.controlConfig_{index}.Ef;
                        modes = obj.controlConfig_{index}.modes;
                        numberOfModes = size(modes,1);
                        pij = obj.controlConfig_{index}.pij;
                        eij = obj.controlConfig_{index}.eij;
                        obj.controlConfig_{index}.F = [];
                        obj.controlConfig_{index}.G = [];
                        obj.controlConfig_{index}.Eqaux = [];
                        obj.controlConfig_{index}.Eraux = [];
                        for it=1:numberOfModes
                            Mt = [];
                            for kt=1:obj.numberOfRotors_
                                Mt = [Mt modes(it,kt)*(obj.rotorLiftCoeff(kt,obj.rotorOperatingPoint_(kt))*cross(obj.rotor_(kt).position,obj.rotor_(kt).orientation)-obj.rotorDragCoeff(kt,obj.rotorOperatingPoint_(kt))*obj.rotorDirection_(kt)*obj.rotor_(kt).orientation)];
                            end
                            obj.controlConfig_{index}.G(:,:,it) = [-obj.inertia()\Mt;zeros(3,obj.numberOfRotors_)];
                            for jt=1:numberOfModes
                                obj.controlConfig_{index}.Ee = [obj.controlConfig_{index}.Ee; [pij(it,jt)*eye(6) zeros(size(Ef(:,:,it)')) eij(it,jt)*eye(6) zeros(size(Ef(:,:,it)'))]'];
                            end
                            obj.controlConfig_{index}.Eqaux = [obj.controlConfig_{index}.Eqaux obj.controlConfig_{index}.Eq(:,:,it)];
                            obj.controlConfig_{index}.Eraux = [obj.controlConfig_{index}.Eraux obj.controlConfig_{index}.Er(:,:,it)];
                        end
                        obj.controlConfig_{index}.Eqaux = obj.controlConfig_{index}.Eqaux';
                        obj.controlConfig_{index}.Eraux = obj.controlConfig_{index}.Eraux';
                        obj.controlConfig_{index}.C = [];
                        for it=1:obj.numberOfRotors_
                            obj.controlConfig_{index}.C = [obj.controlConfig_{index}.C (obj.rotorLiftCoeff(it,obj.rotorOperatingPoint_(it))*cross(obj.rotor_(it).position,obj.rotor_(it).orientation)-obj.rotorDragCoeff(it,obj.rotorOperatingPoint_(it))*obj.rotorDirection_(it)*obj.rotor_(it).orientation)];
                        end
                        obj.controlConfig_{index}.C = -obj.inertia()\obj.controlConfig_{index}.C;
                    end
                    modes = obj.controlConfig_{index}.modes;
                    numberOfModes = size(modes,1);
                    pij = obj.controlConfig_{index}.pij;
                    eij = obj.controlConfig_{index}.eij;
                    
                    Sq = [qe(1),-qe(4),qe(3);
                          qe(4),qe(1),-qe(2);
                          -qe(3),qe(2),qe(1)];
                    F = [];
                    G = [];
                    Ea = [];
                    Eb = [];
                    Ef = obj.controlConfig_{index}.Ef;
                    Eg = obj.controlConfig_{index}.Eg;
                    for it=1:numberOfModes
                        torqueAux = zeros(3,1);
                        for jt=1:obj.numberOfRotors_
                            torqueAux = torqueAux + modes(it,jt)*obj.rotorOrientation(jt)*(obj.previousRotorSpeed(jt)*obj.rotorInertia(jt));
                        end
                        auxA = obj.inertia()*obj.previousAngularVelocity()-torqueAux;
                        auxA = [0 -auxA(3) auxA(2) ; auxA(3) 0 -auxA(1) ; -auxA(2) auxA(1) 0 ];
                        auxA = obj.inertia()\auxA;
                        obj.controlConfig_{index}.F(:,:,it) = [auxA, zeros(3); 0.5*Sq, zeros(3)];
                        sys = ss(obj.controlConfig_{index}.F(:,:,it),obj.controlConfig_{index}.G(:,:,it),eye(6),0);
                        sysD = c2d(sys,obj.controlTimeStep_);

                        F(:,:,it) = sysD.A;
                        G(:,:,it) = sysD.B;
                        
                        for jt=1:numberOfModes
                            Ea = [Ea; [pij(it,jt)*F(:,:,it)' pij(it,jt)*Ef(:,:,it)' eij(it,jt)*F(:,:,it)' eij(it,jt)*Ef(:,:,it)']'];
                            Eb = [Eb; [pij(it,jt)*G(:,:,it)' pij(it,jt)*Eg(:,:,it)' eij(it,jt)*G(:,:,it)' eij(it,jt)*Eg(:,:,it)']'];
                        end
                    end 
                    obj.controlConfig_{index}.Ea = Ea;
                    obj.controlConfig_{index}.Eb = Eb;
                    Ee = obj.controlConfig_{index}.Ee;
                    Eq = obj.controlConfig_{index}.Eqaux;
                    Er = obj.controlConfig_{index}.Eraux;
                                        
                    if ~obj.isRunning()
                        obj.controlConfig_{index}.P = obj.controlConfig_{index}.initialP;
                    end
                    P = obj.controlConfig_{index}.P;
                    k = obj.controlConfig_{index}.k;
                    lambda = obj.controlConfig_{index}.lambda;

                    [~,K,P] = obj.gainPassiveMarkovian(P,Ea,Eb,Ee,Eq,Er,k,lambda);
                    obj.controlConfig_{index}.P = P;
                    x_e = [desiredAngularVelocity-obj.previousAngularVelocity();qe(2:4)'];
                    u = K*x_e
                    C = obj.controlConfig_{index}.C ;
                    torqueAux = zeros(3,1);
                    for it=1:obj.numberOfRotors_
                        torqueAux = torqueAux + obj.rotorOrientation(it)*(obj.previousRotorSpeed(it)*obj.rotorInertia(it));
                    end
                    
                    auxA = obj.inertia()*obj.previousAngularVelocity()-torqueAux;
                    auxA = [0 -auxA(3) auxA(2) ; auxA(3) 0 -auxA(1) ; -auxA(2) auxA(1) 0 ];
                    auxA = obj.inertia()\auxA;
                    
                    attitudeControlOutput = -obj.inertia()*(C*u-desiredAngularAcceleration+auxA*desiredAngularVelocity);  
                case 18 %'Markovian RLQ-R Active Modified'
                    index = 18; 
                    
                    numberOfModes = size(obj.controlConfig_{index}.modes,1);
                    modes = obj.controlConfig_{index}.modes;
                    if ~obj.isRunning()
                        obj.controlConfig_{index}.P = obj.controlConfig_{index}.initialP;
                        obj.controlConfig_{index}.G = [];                        
                        obj.controlConfig_{index}.F = [];
                        obj.controlConfig_{index}.Ep = [];
                        ei = obj.controlConfig_{index}.ei;
                        for it=1:numberOfModes
                            Mt = [];
                            for kt=1:obj.numberOfRotors_
                                Mt = [Mt modes(it,kt)*(obj.rotorLiftCoeff(kt,obj.rotorOperatingPoint_(kt))*cross(obj.rotor_(kt).position,obj.rotor_(kt).orientation)-obj.rotorDragCoeff(kt,obj.rotorOperatingPoint_(kt))*obj.rotorDirection_(kt)*obj.rotor_(kt).orientation)];
                            end
                            obj.controlConfig_{index}.G(:,:,it) = [-obj.inertia()\Mt;zeros(3,obj.numberOfRotors_)];
                            obj.controlConfig_{index}.Ep(:,:,it) = ei(it)*eye(6);
                        end
                    end                    
                    
                    Sq = [qe(1),-qe(4),qe(3);
                          qe(4),qe(1),-qe(2);
                          -qe(3),qe(2),qe(1)];
                    F = [];
                    G = [];
                    for it=1:numberOfModes
                        torqueAux = zeros(3,1);
                        for jt=1:obj.numberOfRotors_
                            torqueAux = torqueAux + modes(it,jt)*obj.rotorOrientation(it)*(obj.previousRotorSpeed(it)*obj.rotorInertia(it)*diagnosis{it}.motorEfficiency*(diagnosis{it}.propEfficiency^2));
                        end
                        auxA = obj.inertia()*obj.previousAngularVelocity()-torqueAux;
                        auxA = [0 -auxA(3) auxA(2) ; auxA(3) 0 -auxA(1) ; -auxA(2) auxA(1) 0 ];
                        auxA = obj.inertia()\auxA;
                        obj.controlConfig_{index}.F(:,:,it) = [auxA, zeros(3); 0.5*Sq, zeros(3)];
                        sys = ss(obj.controlConfig_{index}.F(:,:,it),obj.controlConfig_{index}.G(:,:,it),eye(6),0);
                        sysD = c2d(sys,obj.controlTimeStep_);

                        F(:,:,it) = sysD.A;
                        G(:,:,it) = sysD.B;
                    end                 
                    
                    P = obj.controlConfig_{index}.P;
                    k = obj.controlConfig_{index}.k;
                    Q = obj.controlConfig_{index}.Q;
                    R = obj.controlConfig_{index}.R;
                    H = obj.controlConfig_{index}.H;
                    Ep = obj.controlConfig_{index}.Ep;
                    Ef = obj.controlConfig_{index}.Ef;
                    Eg = obj.controlConfig_{index}.Eg;
                    pij = obj.controlConfig_{index}.pij;
                    mu = obj.controlConfig_{index}.mu;
                    alpha = obj.controlConfig_{index}.alpha;

                    [~,K,Pnext] = obj.gainActiveMarkovian(F,G,P,Q,R,H,Ef,Eg,pij,Ep,k,mu,alpha);
                    obj.controlConfig_{index}.P = Pnext;
                    modesAux = ones(1,obj.numberOfRotors_);
                    for it=1:obj.numberOfRotors_
                        if ~strcmp('motor ok',diagnosis{it}.status{3}) || ~strcmp('prop ok',diagnosis{it}.status{4})
                            modesAux(it) = 0;
                        end
                    end
                    [~,theta,~] = intersect(modes,modesAux,'rows');
                    if isempty(theta)
                        theta = 1;
                    end
                    
                    x_e = [desiredAngularVelocity-obj.previousAngularVelocity();qe(2:4)'];
                    u = K(:,:,theta)*x_e;
                    
                    auxA = obj.controlConfig_{index}.F(1:3,1:3,theta);
                    C = obj.controlConfig_{index}.G(1:3,:,theta);
                    attitudeControlOutput = -obj.inertia()*(C*u-desiredAngularAcceleration+auxA*desiredAngularVelocity);          
            end
        end  
        function allocatorOutput = controlAllocation(obj, controllerOutput, diagnosis)
        %UNTITLED Summary of this function goes here
        %   Detailed explanation goes here
            if isstruct(controllerOutput.attitudeController)
                desiredImpulse = controllerOutput.attitudeController.impulse;
                desiredTorque = controllerOutput.attitudeController.torque;
            else
                desiredImpulse = obj.matrixAbsoluteToBody()*controllerOutput.positionController;
                desiredTorque = controllerOutput.attitudeController;
            end
            %TbPIDD = obj.matrixAbsoluteToBody()*controllerOutput.positionController
            %Tb = desiredImpulse
            switch obj.allocationAlg_
                case 1 %'PI Passive'
                    Mf = [];
                    Mt = [];
                    for it=1:obj.numberOfRotors_
                        Mf = [Mf obj.rotorLiftCoeff(it,obj.rotorOperatingPoint_(it))*obj.rotor_(it).orientation];
                        Mt = [Mt (obj.rotorLiftCoeff(it,obj.rotorOperatingPoint_(it))*cross(obj.rotor_(it).position,obj.rotor_(it).orientation)-obj.rotorDragCoeff(it,obj.rotorOperatingPoint_(it))*obj.rotorDirection_(it)*obj.rotor_(it).orientation)];
                    end
                    H = [Mf;Mt];
                    invH = pinv(H);   
                    [omega_square]=invH*[desiredImpulse;desiredTorque];
                    omega_square(omega_square<0) = 0;
                    for it=1:obj.numberOfRotors_
                        allocatorOutput(it,1) = obj.rotorDirection_(it)*sqrt(omega_square(it));
                    end
                case 2 %'PI Active'
                    Mf = [];
                    Mt = [];
                    for it=1:obj.numberOfRotors_
                        if strcmp('stuck',diagnosis{it}.status{1})
                            Mf = [Mf [0 0 0]'];
                            Mt = [Mt [0 0 0]'];
                        else
                            Mf = [Mf (obj.rotorLiftCoeff(it,obj.rotorOperatingPoint_(it))*obj.rotor_(it).orientation)*(diagnosis{it}.propEfficiency*diagnosis{it}.motorEfficiency^2)];
                            Mt = [Mt (obj.rotorLiftCoeff(it,obj.rotorOperatingPoint_(it))*cross(obj.rotor_(it).position,obj.rotor_(it).orientation)-obj.rotorDragCoeff(it,obj.rotorOperatingPoint_(it))*obj.rotorDirection_(it)*obj.rotor_(it).orientation)*(diagnosis{it}.propEfficiency*diagnosis{it}.motorEfficiency^2)];
                        end
                    end
                    H = [Mf;Mt];
                    invH = pinv(H);                     
                    [omega_square]=invH*[desiredImpulse;desiredTorque];
                    omega_square(omega_square<0) = 0;
                    for it=1:obj.numberOfRotors_
                        allocatorOutput(it,1) = obj.rotorDirection_(it)*sqrt(omega_square(it));
                    end                    
                case 3 %'RPI Passive'
                    tau = [desiredImpulse;desiredTorque];
                    Mf = [];
                    Mt = [];
                    for it=1:obj.numberOfRotors_
                        Mf = [Mf obj.rotorLiftCoeff(it,obj.rotorOperatingPoint_(it))*obj.rotor_(it).orientation];
                        Mt = [Mt (obj.rotorLiftCoeff(it,obj.rotorOperatingPoint_(it))*cross(obj.rotor_(it).position,obj.rotor_(it).orientation)-obj.rotorDragCoeff(it,obj.rotorOperatingPoint_(it))*obj.rotorDirection_(it)*obj.rotor_(it).orientation)];
                    end
                    H = [Mf;Mt];
                    auxH = H;
                    c = zeros(obj.numberOfRotors_,1);
                    inLoop = true;
                    maxSpeeds = ((obj.rotorMaxSpeed((1:obj.numberOfRotors_))).^2)';
                    minSpeeds = ((obj.rotorMinSpeed((1:obj.numberOfRotors_))).^2)';
                    while inLoop
                        [omega_square] = -c + pinv(auxH)*(tau+H*c);
                        omega_square(abs(omega_square)<1) = 0;
                        if all(omega_square<=maxSpeeds) && all(omega_square>=minSpeeds)
                            inLoop = false;
                        else
                            for it=1:obj.numberOfRotors_
                                if omega_square(it)<=minSpeeds(it)
                                    c(it) = -minSpeeds(it)*0.999999999;
                                    auxH(:,it) = zeros(size(auxH(:,it)));
                                elseif omega_square(it)>=maxSpeeds(it)
                                    c(it) = -maxSpeeds(it)*0.999999999;
                                    auxH(:,it) = zeros(size(auxH(:,it)));   
                                end
                            end
                        end
                    end
                    for it=1:obj.numberOfRotors_
                        allocatorOutput(it,1) = obj.rotorDirection_(it)*sqrt(omega_square(it));
                    end      
                case 4 %'RPI Active'
                    tau = [desiredImpulse;desiredTorque];
                    Mf = [];
                    Mt = [];
                    for it=1:obj.numberOfRotors_
                        if strcmp('stuck',diagnosis{it}.status{1})
                            Mf = [Mf [0 0 0]'];
                            Mt = [Mt [0 0 0]'];
                        else
                            Mf = [Mf (obj.rotorLiftCoeff(it,obj.rotorOperatingPoint_(it))*obj.rotor_(it).orientation)*(diagnosis{it}.propEfficiency*diagnosis{it}.motorEfficiency^2)];
                            Mt = [Mt (obj.rotorLiftCoeff(it,obj.rotorOperatingPoint_(it))*cross(obj.rotor_(it).position,obj.rotor_(it).orientation)-obj.rotorDragCoeff(it,obj.rotorOperatingPoint_(it))*obj.rotorDirection_(it)*obj.rotor_(it).orientation)*(diagnosis{it}.propEfficiency*diagnosis{it}.motorEfficiency^2)];
                        end
                    end
                    H = [Mf;Mt];
                    auxH = H;
                    c = zeros(obj.numberOfRotors_,1);
                    inLoop = true;
                    for it=1:obj.numberOfRotors_
                        if strcmp('stuck',diagnosis{it}.status{1})
                            maxSpeeds(it,:) = 0;
                            minSpeeds(it,:) = 0;
                        else
                            maxSpeeds(it,:) = (obj.rotorMaxSpeed(it)*diagnosis{it}.motorEfficiency)^2;
                            minSpeeds(it,:) = (obj.rotorMinSpeed(it)*diagnosis{it}.motorEfficiency)^2;
                        end
                    end
                    while inLoop
                        [omega_square] = -c + pinv(auxH)*(tau+H*c);
                        omega_square(abs(omega_square)<1) = 0;
                        if all(omega_square<=maxSpeeds) && all(omega_square>=minSpeeds)
                            inLoop = false;
                        else
                            for it=1:obj.numberOfRotors_
                                if omega_square(it)<=minSpeeds(it)
                                    c(it) = -minSpeeds(it)*0.999999999;
                                    auxH(:,it) = zeros(size(auxH(:,it)));
                                elseif omega_square(it)>=maxSpeeds(it)
                                    c(it) = -maxSpeeds(it)*0.999999999;
                                    auxH(:,it) = zeros(size(auxH(:,it)));   
                                end
                            end
                        end
                    end
                    for it=1:obj.numberOfRotors_
                        allocatorOutput(it,1) = obj.rotorDirection_(it)*sqrt(omega_square(it));
                    end  
%                      allocatorOutput
                case 5 %'Adaptive'
                    index = 5;
                    x = [obj.matrixAbsoluteToBody()*obj.previousVelocity();obj.previousAngularVelocity()];
                    if ~obj.isRunning()
                        obj.allocationConfig_{index}.P = lyap(obj.allocationConfig_{index}.Am',-eye(6));
                        obj.allocationConfig_{index}.Xest = x;
                        obj.allocationConfig_{index}.X = x;
                        
                        Mf = [];
                        Mt = [];
                        torqueAux = zeros(3,1);
                        for it=1:obj.numberOfRotors_
                            Mf = [Mf (obj.rotorLiftCoeff(it,obj.rotorOperatingPoint_(it))*obj.rotor_(it).orientation)];
                            Mt = [Mt (obj.rotorLiftCoeff(it,obj.rotorOperatingPoint_(it))*cross(obj.rotor_(it).position,obj.rotor_(it).orientation)-obj.rotorDragCoeff(it,obj.rotorOperatingPoint_(it))*obj.rotorDirection_(it)*obj.rotor_(it).orientation)];
                            torqueAux = torqueAux + obj.rotorOrientation(it)*(obj.previousRotorSpeed(it).*obj.rotorInertia(it));
                        end                    
                        auxA = obj.inertia()*obj.previousAngularVelocity()-torqueAux;
                        auxA = obj.inertia()\[0 -auxA(3) auxA(2) ; auxA(3) 0 -auxA(1) ; -auxA(2) auxA(1) 0 ];
                        
                        obj.allocationConfig_{index}.Apest = blkdiag(-obj.friction()/obj.mass(),auxA);
                        obj.allocationConfig_{index}.A0 = obj.allocationConfig_{index}.Apest;
                        obj.allocationConfig_{index}.Bpest = [Mf/obj.mass();obj.inertia()\Mt];
                        %obj.allocationConfig_{index}.B0 = [Mf/obj.mass();obj.inertia()\Mt];
                        obj.allocationConfig_{index}.nullMt0 = null(Mt);
                        obj.allocationConfig_{index}.Mf0 = Mf;
                        obj.allocationConfig_{index}.pinvMt0 = pinv(Mt);
                    end
                    P = obj.allocationConfig_{index}.P;
%                     previousXest = obj.allocationConfig_{index}.Xest;
                    previousX = obj.allocationConfig_{index}.X;
                    previousApest = obj.allocationConfig_{index}.Apest;
                    previousBpest = obj.allocationConfig_{index}.Bpest;
                    R = obj.allocationConfig_{index}.R;
                    Q = obj.allocationConfig_{index}.Q;
                    nullMt0 = obj.allocationConfig_{index}.nullMt0;
                    maxSpeeds = ((obj.rotorMaxSpeed(1:obj.numberOfRotors_)).^2)';
                    minSpeeds = ((obj.rotorMinSpeed(1:obj.numberOfRotors_)).^2)';
                    op = (maxSpeeds+minSpeeds)/2; % in the middle of the squared rotor speed range, to best maneuverability
                    pinvMt0 = obj.allocationConfig_{index}.pinvMt0;
                    Mf0 = obj.allocationConfig_{index}.Mf0;
                    
                    previousU = obj.previousInput().^2;
%                     N = nullMt0;
%                     v = (N'*(R*N+Mf0'*Q*Mf0*N))\(N'*(R*op+Mf0'*Q*desiredImpulse));
%                     b2 = N*v;
%                     utau = pinvMt0*desiredTorque;
%                     c = (desiredImpulse-Mf0*utau)'*Mf0*b2/(norm(Mf0*b2)^2);
%                     omega_square = utau+b2*c;                    
%                     lessIndex = omega_square<minSpeeds;
%                     greaterIndex = omega_square>maxSpeeds;
%                     omega_square(lessIndex) = minSpeeds(lessIndex);
%                     omega_square(greaterIndex) = maxSpeeds(greaterIndex);
%                     u = omega_square;
%                     obj.allocationConfig_{index}.X = x;

                    Bpest = obj.allocationConfig_{index}.Bpest
                    Mf = obj.mass()*Bpest(1:3,:);
                    Mt = obj.inertia()*Bpest(4:6,:);
                    N = null(Mt);
                    v = (N'*(R*N+Mf'*Q*Mf*N))\(N'*(R*op+Mf'*Q*desiredImpulse));
                    b2 = N*v;
                    utau = pinv(Mt)*desiredTorque;
                    c = (desiredImpulse-Mf*utau)'*Mf*b2/(norm(Mf*b2)^2);
                    omega_square = utau+b2*c;
                    
                    lessIndex = omega_square<minSpeeds;
                    greaterIndex = omega_square>maxSpeeds;
                    omega_square(lessIndex) = minSpeeds(lessIndex);
                    omega_square(greaterIndex) = maxSpeeds(greaterIndex);
                    for it=1:obj.numberOfRotors_
                        allocatorOutput(it,1) = obj.rotorDirection_(it)*sqrt(omega_square(it));
                    end
                    u = omega_square;
                    
%                     Am = obj.allocationConfig_{index}.Am;
%                     sys = ss(Am,[previousApest-Am,previousBpest,[obj.matrixAbsoluteToBody(),zeros(3)]'],eye(6),0);
%                     sysD = c2d(sys,obj.controlTimeStep_);
% 
%                     Ad = sysD.A;
%                     Bd = sysD.B;
%                     
%                     obj.allocationConfig_{index}.Xest = Ad*previousXest+Bd*[previousX;previousU;[0;0;-9.81]];
                    obj.allocationConfig_{index}.Xest = obj.controlTimeStep_*(previousApest*previousX+previousBpest*previousU+[obj.matrixAbsoluteToBody()*[0;0;-9.81];zeros(3,1)]) + obj.allocationConfig_{index}.Xest;
%                     x=x
%                     xest = obj.allocationConfig_{index}.Xest
                    error = x-obj.allocationConfig_{index}.Xest;
%                     A0 = obj.allocationConfig_{index}.A0
                    obj.allocationConfig_{index}.Apest = -P*error*x'+obj.allocationConfig_{index}.Apest;
%                     Aest = obj.allocationConfig_{index}.Apest
%                     B0 = obj.allocationConfig_{index}.B0
                    obj.allocationConfig_{index}.Bpest = -P*error*u'+obj.allocationConfig_{index}.Bpest;
%                     Best = obj.allocationConfig_{index}.Bpest
                case 6 % 'CLS Passive'
                    Mf = [];
                    Mt = [];
                    for it=1:obj.numberOfRotors_
                        Mf = [Mf obj.rotorLiftCoeff(it,obj.rotorOperatingPoint_(it))*obj.rotor_(it).orientation];
                        Mt = [Mt (obj.rotorLiftCoeff(it,obj.rotorOperatingPoint_(it))*cross(obj.rotor_(it).position,obj.rotor_(it).orientation)-obj.rotorDragCoeff(it,obj.rotorOperatingPoint_(it))*obj.rotorDirection_(it)*obj.rotor_(it).orientation)];
                    end
                    H = [Mf;Mt];
                    
                    maxSpeeds = ((obj.rotorMaxSpeed(1:obj.numberOfRotors_)).^2)';
                    minSpeeds = ((obj.rotorMinSpeed(1:obj.numberOfRotors_)).^2)';
                    omega_square = lsqlin(H,[desiredImpulse;desiredTorque],[],[],[],[],minSpeeds,maxSpeeds)
                    
                    for it=1:obj.numberOfRotors_
                        allocatorOutput(it,1) = obj.rotorDirection_(it)*sqrt(omega_square(it));
                    end
                case 7 % 'Passive NMAC'
                    index = 7;
                    if ~obj.isRunning()
                        obj.allocationConfig_{index}.Mf = [];
                        obj.allocationConfig_{index}.Mt = [];
                        for it=1:obj.numberOfRotors_
                            obj.allocationConfig_{index}.Mf = [obj.allocationConfig_{index}.Mf obj.rotorLiftCoeff(it,obj.rotorOperatingPoint_(it))*obj.rotor_(it).orientation];
                            obj.allocationConfig_{index}.Mt = [obj.allocationConfig_{index}.Mt (obj.rotorLiftCoeff(it,obj.rotorOperatingPoint_(it))*cross(obj.rotor_(it).position,obj.rotor_(it).orientation)-obj.rotorDragCoeff(it,obj.rotorOperatingPoint_(it))*obj.rotorDirection_(it)*obj.rotor_(it).orientation)];
                        end
                        obj.allocationConfig_{index}.maxSpeeds = ((obj.rotorMaxSpeed(1:obj.numberOfRotors_)).^2)';
                        obj.allocationConfig_{index}.minSpeeds = ((obj.rotorMinSpeed(1:obj.numberOfRotors_)).^2)';
                        obj.allocationConfig_{index}.op = (obj.allocationConfig_{index}.maxSpeeds+obj.allocationConfig_{index}.minSpeeds)/2; % in the middle of the squared rotor speed range, to best maneuverability
                        obj.allocationConfig_{index}.N = null(obj.allocationConfig_{index}.Mt);
                        obj.allocationConfig_{index}.pinvMt = pinv(obj.allocationConfig_{index}.Mt);
                        
                    end
                    Mf = obj.allocationConfig_{index}.Mf;
                    Mt = obj.allocationConfig_{index}.Mt;
                    pinvMt = obj.allocationConfig_{index}.pinvMt;
                    maxSpeeds = obj.allocationConfig_{index}.maxSpeeds;
                    minSpeeds = obj.allocationConfig_{index}.minSpeeds;
                    op = obj.allocationConfig_{index}.op;
                    N = obj.allocationConfig_{index}.N;
                    R = obj.allocationConfig_{index}.R;
                    Q = obj.allocationConfig_{index}.Q;
                    v = (N'*(R*N+Mf'*Q*Mf*N))\(N'*(R*op+Mf'*Q*desiredImpulse));
                    b2 = N*v;
                    utau = pinvMt*desiredTorque;
                    c = (desiredImpulse-Mf*utau)'*Mf*b2/(norm(Mf*b2)^2);
                    omega_square = utau+b2*c;
                    
%                     desiredImpulse
%                     desiredTorque
%                     realizedImpulse = Mf*omega_square
%                     realizedTorque = Mt*omega_square
                    
                    lessIndex = omega_square<minSpeeds;
                    greaterIndex = omega_square>maxSpeeds;
                    omega_square(lessIndex) = minSpeeds(lessIndex);
                    omega_square(greaterIndex) = maxSpeeds(greaterIndex);
                    for it=1:obj.numberOfRotors_
                        allocatorOutput(it,1) = obj.rotorDirection_(it)*sqrt(omega_square(it));
                    end
                case 8 % 'Active NMAC'
                    index = 8;
                    Mf = [];
                    Mt = [];
                    for it=1:obj.numberOfRotors_
                        if strcmp('stuck',diagnosis{it}.status{1})
                            Mf = [Mf [0 0 0]'];
                            Mt = [Mt [0 0 0]'];
                        else
                            Mf = [Mf (obj.rotorLiftCoeff(it,obj.rotorOperatingPoint_(it))*obj.rotor_(it).orientation)*(diagnosis{it}.propEfficiency*diagnosis{it}.motorEfficiency^2)];
                            Mt = [Mt (obj.rotorLiftCoeff(it,obj.rotorOperatingPoint_(it))*cross(obj.rotor_(it).position,obj.rotor_(it).orientation)-obj.rotorDragCoeff(it,obj.rotorOperatingPoint_(it))*obj.rotorDirection_(it)*obj.rotor_(it).orientation)*(diagnosis{it}.propEfficiency*diagnosis{it}.motorEfficiency^2)];
                        end
                    end
                    maxSpeeds = zeros(obj.numberOfRotors_,1);
                    minSpeeds = zeros(obj.numberOfRotors_,1);
                    for it=1:obj.numberOfRotors_
                        if strcmp('stuck',diagnosis{it}.status{1})
                            maxSpeeds(it,:) = 0;
                            minSpeeds(it,:) = 0;
                        else
                            maxSpeeds(it,:) = (obj.rotorMaxSpeed(it)*diagnosis{it}.motorEfficiency)^2;
                            minSpeeds(it,:) = (obj.rotorMinSpeed(it)*diagnosis{it}.motorEfficiency)^2;
                        end
                    end
                    op = (maxSpeeds+minSpeeds)/2; % in the middle of the squared rotor speed range, to best maneuverability
                    N = null(Mt);
                    R = obj.allocationConfig_{index}.R;
                    Q = obj.allocationConfig_{index}.Q;
                    v = (N'*(R*N+Mf'*Q*Mf*N))\(N'*(R*op+Mf'*Q*desiredImpulse));
                    b2 = N*v;
                    utau = pinv(Mt)*desiredTorque;
                    c = (desiredImpulse-Mf*utau)'*Mf*b2/(norm(Mf*b2)^2);
                    omega_square = utau+b2*c;
                    
                    lessIndex = omega_square<minSpeeds;
                    greaterIndex = omega_square>maxSpeeds;
                    omega_square(lessIndex) = minSpeeds(lessIndex);
                    omega_square(greaterIndex) = maxSpeeds(greaterIndex);
                    for it=1:obj.numberOfRotors_
                        allocatorOutput(it,1) = obj.rotorDirection_(it)*sqrt(omega_square(it));
                    end
                otherwise
                    allocatorOutput = controllerOutput.attitudeController; 
            end
            switch obj.simEffects_{1}
                case 'motor dynamics on'  
                    allocatorOutput = ([obj.rotor_(:).Rm].*[obj.rotorDragCoeff(1:obj.numberOfRotors_,obj.rotorOperatingPoint_)].*allocatorOutput'.*abs(allocatorOutput')./[obj.rotor_(:).Kt]+60*allocatorOutput'./([obj.rotor_(:).Kv]*2*pi))';
                otherwise
                    % does nothing
            end
        end  
        function result = canRunControl(obj)
        %UNTITLED Summary of this function goes here
        %   Detailed explanation goes here
            result = true;
            if isempty(obj.trajectoryMap_.endTime)
                error('Cannot run. Set trajectory.')
                result = false;
            end
            if isempty(obj.rotorDirection_) || (length(obj.rotorDirection_)~=obj.numberOfRotors_) || ~any(obj.rotorDirection_)
                error('Cannot run. Specify rotor directions for each rotor in multicopter.')
                result = false;
            end
            if isempty(obj.rotorOperatingPoint_) || (length(obj.rotorOperatingPoint_)~=obj.numberOfRotors_) || any(obj.rotorOperatingPoint_==0)
                error('Cannot run. Specify rotor operating points greater than zero for each rotor in multicopter.')
                result = false;
            end
            if isempty(obj.controlAlg_)
                error('Cannot run. Specify controller algorithm to be used.')
                result = false;
            end
            if isempty(obj.allocationAlg_)
                error('Cannot run. Specify control allocation algorithm to be used.')
                result = false;
            end            
            if isempty(obj.attReferenceAlg_) || strcmp('none',obj.attReferenceAlg_) || strcmp('None',obj.attReferenceAlg_)
                error('Cannot run. Specify control allocation algorithm for attitude reference calculation to be used.')
                result = false;
            end
            if isempty(obj.controlConfig_) 
                error('Cannot run. Configure attitude controller parameters!')
                result = false;
            else
                if length(obj.controlConfig_)<obj.controlAlg_
                    error('Cannot run. %s attitude controller not configured!',obj.attitudeControllers_{obj.controlAlg_})
                    result = false;
                end
                if isempty(obj.controlConfig_{obj.controlAlg_})
                    error('Cannot run. %s attitude controller not configured!',obj.attitudeControllers_{obj.controlAlg_})
                    result = false;
                end
            end
            if isempty(obj.positionControlConfig_.kp) || isempty(obj.positionControlConfig_.kd) || isempty(obj.positionControlConfig_.kdd) 
                error('Cannot run. Configure position controller parameters!')
                result = false;
            end
            if (isempty(obj.fddConfig_.hitRate) || isempty(obj.fddConfig_.delay)) && (~isempty(strfind(obj.attitudeControllers_{obj.controlAlg_},'active')) || ~isempty(strfind(obj.attitudeControllers_{obj.controlAlg_},'Active')))
                error('Cannot run. Configure FDD correctly.');
                result = false;
            end
            if (isempty(obj.fddConfig_.hitRate) || isempty(obj.fddConfig_.delay)) && (~isempty(strfind(obj.controlAllocators_{obj.allocationAlg_},'active')) || ~isempty(strfind(obj.controlAllocators_{obj.allocationAlg_},'Active')))
                error('Cannot run. Configure FDD correctly.');
                result = false;
            end
            if (isempty(obj.fddConfig_.hitRate) || isempty(obj.fddConfig_.delay)) && (~isempty(strfind(obj.controlAllocators_{obj.attReferenceAlg_},'active')) || ~isempty(strfind(obj.controlAllocators_{obj.attReferenceAlg_},'Active')))
                error('Cannot run. Configure FDD correctly.');
                result = false;
            end
            if (~isempty(strfind(obj.controlAllocators_{obj.attReferenceAlg_},'none')) || ...
                    ~isempty(strfind(obj.controlAllocators_{obj.attReferenceAlg_},'None')))
                error('Cannot run. Choose allocation algorithm for attitude reference.');
                result = false;
            end
            if (~isempty(strfind(obj.attitudeControllers_{obj.controlAlg_},'Direct')) || ...
                    ~isempty(strfind(obj.attitudeControllers_{obj.controlAlg_},'direct'))) && ...
                    (isempty(strfind(obj.controlAllocators_{obj.allocationAlg_},'none')) && ...
                     isempty(strfind(obj.controlAllocators_{obj.allocationAlg_},'None')))
                error('Cannot run. Attitude controller and control allocation algorithms do not match.');
                result = false;
            end
            
            if isempty(obj.automationConfig_.time) && obj.verbose_ == true
                warning('No automation command configured.');
            end
            
            if ~isempty(strfind(obj.controlAllocators_{obj.allocationAlg_},'Adaptive')) || ...
                    ~isempty(strfind(obj.controlAllocators_{obj.attReferenceAlg_},'Adaptive'))
                if isempty(obj.allocationConfig_)
                    error('Cannot run. Configure adaptive control allocation.')
                    result = false;
                else
                    if isempty(obj.allocationConfig_{obj.attReferenceAlg_})
                        error('Cannot run. Configure adaptive control allocation.')
                        result = false;                    
                    end
                end
            end
            
            if ~isempty(strfind(obj.controlAllocators_{obj.allocationAlg_},'Passive NMAC')) || ...
                    ~isempty(strfind(obj.controlAllocators_{obj.attReferenceAlg_},'Passive NMAC'))
                if isempty(obj.allocationConfig_)
                    error('Cannot run. Configure Passive NMAC control allocation and/or attitude reference.')
                    result = false;
                else
                    if isempty(obj.allocationConfig_{obj.attReferenceAlg_})
                        error('Cannot run. Configure Passive NMAC control allocation and/or attitude reference.')
                        result = false;                    
                    end
                end
            end
            
            if ~isempty(strfind(obj.controlAllocators_{obj.allocationAlg_},'Active NMAC')) || ...
                    ~isempty(strfind(obj.controlAllocators_{obj.attReferenceAlg_},'Active NMAC'))
                if isempty(obj.allocationConfig_)
                    error('Cannot run. Configure Active NMAC control allocation and/or attitude reference.')
                    result = false;
                else
                    if isempty(obj.allocationConfig_{obj.attReferenceAlg_})
                        error('Cannot run. Configure Active NMAC control allocation and/or attitude reference.')
                        result = false;                    
                    end
                end
            end
            
            if isempty(strfind(obj.controlAllocators_{obj.allocationAlg_},'Adaptive')) && ...
                    ~isempty(strfind(obj.controlAllocators_{obj.attReferenceAlg_},'Adaptive'))
                error('Cannot run. Adaptive reference attitude must run with adaptive control allocation.')                    
            end
        end  
        function evalCommands(obj, indexes)
        %UNTITLED Summary of this function goes here
        %   Detailed explanation goes here
            for it=indexes
                for j=1:length(obj.automationConfig_.commands{it})
                    eval(['obj.',obj.automationConfig_.commands{it}{j}]);
                end
            end
        end
        %Auxiliary methods
        function length = pathLength(obj,points)
        %UNTITLED Summary of this function goes here
        %   Detailed explanation goes here
            length = 0;
            for it=1:size(points,2)-1
                length = length + sqrt(sum((points(:,it+1)-points(:,it)).^2));
            end
        end
        function updateMetrics(obj)
        %UNTITLED Summary of this function goes here
        %   Detailed explanation goes here            
            [obj.metrics_.missionSuccess,successTime]   = obj.verifyMissionSuccess();
            obj.metrics_.pathLength                     = obj.pathLength(obj.log_.position);
            obj.metrics_.desiredPathLength              = obj.pathLength(obj.trajectory_.position);
            obj.metrics_.pathLengthRatio                = obj.metrics_.pathLength/obj.metrics_.desiredPathLength;
            obj.metrics_.pathTimeRatio                  = successTime/obj.trajectoryMap_.endTime(end);
            speed = sqrt(sum(obj.log_.velocity.^2,1));
            obj.metrics_.averageSpeed                   = mean(speed);
            obj.metrics_.maxSpeed                       = max(speed);
            for it=1:size(obj.trajectory_.time,2)
                difference = abs(obj.log_.time-obj.trajectory_.time(it));
                [~, index(it)] = min(difference);
            end
            positionError = sqrt(sum((obj.log_.position(:,index)-obj.trajectory_.position).^2,1));
            positionError(isnan(positionError)) = 10*obj.metrics_.simulationEndError;
            obj.metrics_.meanPositionError              = mean(positionError);
            obj.metrics_.RMSPositionError               = rms(positionError);
            obj.metrics_.maxPositionError               = max(positionError);
            angularError = sqrt(sum((obj.toEuler(obj.log_.attitude(:,index))-obj.toEuler(obj.trajectory_.attitude)).^2,1));
            angularError(isnan(angularError)) = 10*obj.metrics_.missionAngularPrecision*pi/180;
            obj.metrics_.meanAngularError           = mean(angularError);
            obj.metrics_.RMSAngularError               = rms(angularError);
            obj.metrics_.maxAngularError               = max(angularError);    
            powerAux = obj.log_.power;
            powerAux(isnan(powerAux)) = max(obj.log_.power);
            obj.metrics_.energy                         = trapz(obj.log_.time,abs(powerAux));
            obj.metrics_.RMSPower                       = rms(powerAux);
            obj.metrics_.maxPower                       = max(abs(powerAux));
        end
        function result = inputsOK(obj,input,start,nelements)
            result = true;
            if ~ischar(input{start})
                result = false;
            end
            if ~(length(input)>=(start+nelements))
                result = false;
            end
            if result==true
                for it=1:nelements
                    if isempty(input{start+it}) || ischar(input{start+it})
                        result = false;
                    end
                end
            end
        end
        function it = configureRLQ(obj,rlqIndex,varArgs,it)
            if obj.inputsOK(varArgs,it,8)
                it = it+1;
                if isnumeric(varArgs{it}) && sum(size(varArgs{it}))>2
                    if isnumeric(varArgs{it+1}) && sum(size(varArgs{it+1}))>2
                        if isnumeric(varArgs{it+2}) && sum(size(varArgs{it+2}))>2
                            if isnumeric(varArgs{it+3}) && sum(size(varArgs{it+3}))>2
                                if isnumeric(varArgs{it+4}) && sum(size(varArgs{it+4}))>2
                                    if isnumeric(varArgs{it+5}) && sum(size(varArgs{it+5}))>2
                                        if isnumeric(varArgs{it+6}) && sum(size(varArgs{it+6}))<=2 && varArgs{it+6}>0
                                            if isnumeric(varArgs{it+7}) && sum(size(varArgs{it+7}))<=2 && varArgs{it+7}>=1
                                                obj.controlConfig_{rlqIndex}.initialP = varArgs{it};
                                                obj.controlConfig_{rlqIndex}.Q = varArgs{it+1};
                                                obj.controlConfig_{rlqIndex}.R = varArgs{it+2};
                                                obj.controlConfig_{rlqIndex}.Ef = varArgs{it+3};
                                                obj.controlConfig_{rlqIndex}.Eg = varArgs{it+4};
                                                obj.controlConfig_{rlqIndex}.H = varArgs{it+5};
                                                obj.controlConfig_{rlqIndex}.mu = varArgs{it+6};
                                                obj.controlConfig_{rlqIndex}.alpha = varArgs{it+7};
                                                
                                                if obj.verbose_ == true
                                                    disp([obj.attitudeControllers_{rlqIndex},' Attitude controller P initial state variance gain set to: ']);
                                                    disp(obj.controlConfig_{rlqIndex}.initialP);
                                                    disp([obj.attitudeControllers_{rlqIndex},' Attitude controller Q state gain set to: '])
                                                    disp(obj.controlConfig_{rlqIndex}.Q);
                                                    disp([obj.attitudeControllers_{rlqIndex},' Attitude controller R input gain set to: '])
                                                    disp(obj.controlConfig_{rlqIndex}.R);
                                                    disp([obj.attitudeControllers_{rlqIndex},' Attitude controller uncertainty Ef set to: '])
                                                    disp(obj.controlConfig_{rlqIndex}.Ef);
                                                    disp([obj.attitudeControllers_{rlqIndex},' Attitude controller uncertainty Eg set to: '])
                                                    disp(obj.controlConfig_{rlqIndex}.Eg);
                                                    disp([obj.attitudeControllers_{rlqIndex},' Attitude controller uncertainty H set to: '])
                                                    disp(obj.controlConfig_{rlqIndex}.H);
                                                    disp([obj.attitudeControllers_{rlqIndex},' Attitude controller mu parameter set to: '])
                                                    disp(obj.controlConfig_{rlqIndex}.mu);
                                                    disp([obj.attitudeControllers_{rlqIndex},' Attitude controller alpha parameter set to: '])
                                                    disp(obj.controlConfig_{rlqIndex}.alpha);
                                                end
                                            else
                                                warning('Attitude controller alpha parameter must be a numeric scalar greater than or equal 1. Skipping this controller configuration');
                                            end
                                        else
                                            warning('Attitude controller mu parameter must be a numeric scalar greater than 0. Skipping this controller configuration');
                                        end
                                    else
                                        warning('Attitude controller uncertainty H must be a numeric matrix. Skipping this controller configuration');
                                    end
                                else
                                    warning('Attitude controller uncertainty Eg must be a numeric matrix. Skipping this controller configuration');
                                end
                            else
                                warning('Attitude controller uncertainty Ef must be a numeric matrix. Skipping this controller configuration');
                            end
                        else
                            warning('Attitude controller R input gain must be a numeric matrix. Skipping this controller configuration');
                        end
                    else
                        warning('Attitude controller Q state gain must be a numeric matrix. Skipping this controller configuration');
                    end
                else
                    warning('Attitude controller initial state variance must be a numeric matrix. Skipping this controller configuration');
                end
                it = it+7;                
            else
                warning('Missing argument for attitude %s controller. Skipping this controller configuration.',obj.attitudeControllers_{rlqIndex});
            end
        end
        function [L,K,P] = gainRLQR(obj,P,R,Q,mu,alpha,F,G,H,Ef,Eg)
        %GAINRLQR Calculates RLQ-R gain and ricatti
        %   Calculates the optimal gains for x(i+1)* and u(i)* in the robust case
        %   considering parametric uncertainties H, Ef and Eg. Returns also the
        %   ricatti P(i) for the P(i+1) and other parameters.
            n = size(F,1);
            m = size(G,2);
            k = size(H,2);
            l = size(Ef,1);

            lambda = alpha*norm(mu*(H'*H));
            left = [P^(-1), zeros(n,2*n+m+l), eye(n), zeros(n,m)
                    zeros(m,n), R^(-1), zeros(m,3*n+l), eye(m)
                    zeros(n,n+m),Q^(-1), zeros(n,2*n+l+m)
                    zeros(n,2*n+m), eye(n)/mu - (H*H')/lambda, zeros(n,l), eye(n), -G
                    zeros(l,3*n+m), eye(l)/lambda, zeros(l,n), -Eg
                    eye(n), zeros(n,m+n), eye(n), zeros(n,l+n+m)
                    zeros(m,n), eye(m), zeros(m,n), -G', -Eg', zeros(m,n+m)];
            right = [zeros(n+m,n);-eye(n);F;Ef;zeros(n+m,n)];
            gain = left\right;
            L = gain(3*n+m+l+1:4*n+m+l,:);
            K = gain(4*n+m+l+1:end,:);

            border = [eye(n);zeros(l,n)]*L-[G;Eg]*K-[F;Ef];
            middle = [eye(n)/mu - (H*H')/lambda, zeros(n,l); zeros(l,n), eye(l)/lambda];

            P = L'*P*L+K'*R*K+Q+(border'/middle)*border;
        end
        function [L,K,P] = gainPassiveMarkovian(obj,P,Ea,Eb,Ee,Eq,Er,k,lambda)
            %GAINRLQR Calculates RLQ-R gain and ricatti
        %   Calculates the optimal gains for x(i+1)* and u(i)* in the robust case
        %   considering parametric uncertainties H, Ef and Eg. Returns also the
        %   ricatti P(i) for the P(i+1) and other parameters.
            n = size(Ea,2);
            m = size(Eb,2);
            r = size(Er,1);
            Ea = k*Ea;
            Eb = k*Eb;
            Ee = k*Ee;
            Eq = k*Eq;
            Er = k*Er;

            A = [zeros(r,n);-Eq;Ea];
            B = [-Er;zeros(size(Eq,1),m);Eb];
            E = [zeros(r+size(Eq,1),n);Ee];
            sigma = eye(size(A,1))/lambda;
            left = [zeros(n,n+m+n)
                    zeros(size(A,1),n+m), A
                    eye(n) zeros(n,m+n)
                    zeros(m,n), eye(m), zeros(m,n)]';
            topCenter = blkdiag(inv(P),sigma);
            centerAux = [eye(n), zeros(n,m)
                         E,-B];
            center = [topCenter, centerAux
                      centerAux',zeros(n+m,n+m)];
            right = [zeros(n,n);A;zeros(n+m,n)];
            gain = left*(center\right);
            L = gain(1:n,:);
            K = gain(n+1:n+m,:);
            P = gain(n+m+1:end,:);
        end
        function [L,K,Pnext] = gainActiveMarkovian(obj,F,G,P,Q,R,H,Ef,Eg,pij,Ep,k,mu,alpha)
            %GAINRLQR Calculates RLQ-R gain and ricatti
        %   Calculates the optimal gains for x(i+1)* and u(i)* in the robust case
        %   considering parametric uncertainties H, Ef and Eg. Returns also the
        %   ricatti P(i) for the P(i+1) and other parameters.
            n = size(F,2);
            m = size(G,2);
            s = size(F,3);
            l = size(Ef,1);
            Ep = k*Ep;
            Ef = k*Ef;
            Eg = k*Eg;
            
            for it=1:s
                psi = zeros(n);          
                for jt = 1:s  
                    psi = psi + pij(it,jt) * P(:,:,jt);
                end
                Fhat = [F(:,:,it);zeros(n,n);Ef(:,:,it)];
                Ghat = [G(:,:,it);zeros(n,m);Eg(:,:,it)];
                Ihat = [eye(n);Ep(:,:,it);zeros(size(Ef(:,:,it)))];
                lambda = (1+alpha)*norm(mu*H(:,:,it)'*H(:,:,it));
                theta = eye(n)/mu-H(:,:,it)*H(:,:,it)'/lambda;
                sigma = blkdiag(theta, eye(n)/lambda, eye(l)/lambda);
                
                left = [zeros(n+m,n+m+n)
                        zeros(n,n+m), -eye(n)
                        zeros(size(Fhat,1),n+m), Fhat
                        eye(n), zeros(n,m+n)
                        zeros(m,n), eye(m), zeros(m,n)]';
                auxCenter = [eye(n+m)
                             zeros(n,n+m)
                             Ihat, -Ghat];
                topCenter = blkdiag(inv(psi),inv(R(:,:,it)),inv(Q(:,:,it)),sigma);
                center = [topCenter,auxCenter
                          auxCenter',zeros(n+m,n+m)];
                right = [zeros(n+m,n);-eye(n);Fhat;zeros(n+m,n)];
                gain = left*(center\right);
                L(:,:,it) = gain(1:n,:);
                K(:,:,it) = gain(n+1:n+m,:);
                Pnext(:,:,it) = gain(n+m+1:end,:);
            end
        end
        function it = configureSOSMC(obj,index,varArgs,it)
            if obj.inputsOK(varArgs,it,3)
                it = it+1;
                if isnumeric(varArgs{it}) && isdiag(varArgs{it}) && all(diag(varArgs{it})>=0)
                    if isnumeric(varArgs{it+1}) %& all(varArgs{it+1}>=0)
                        if isnumeric(varArgs{it+2}) %& all(varArgs{it+2}>=0)
                            obj.controlConfig_{index}.c = varArgs{it};
                            obj.controlConfig_{index}.lambda = varArgs{it+1};
                            obj.controlConfig_{index}.alpha = varArgs{it+2};
                            
                            if obj.verbose_ == true
                                disp([obj.attitudeControllers_{index},' Attitude controller c error gain set to: ']);
                                disp(obj.controlConfig_{index}.c);
                                disp([obj.attitudeControllers_{index},' Attitude controller Lambda gain set to: '])
                                disp(obj.controlConfig_{index}.lambda);
                                disp([obj.attitudeControllers_{index},' Attitude controller Alpha gain set to: '])
                                disp(obj.controlConfig_{index}.alpha);
                            end
                        else
                            warning('Attitude controller Alpha gain must be a numeric matrix. Skipping this controller configuration');
                        end
                    else
                        warning('Attitude controller Lambda gain must be a numeric matrix. Skipping this controller configuration');
                    end
                else
                    warning('Attitude controller c gain must be a positive numeric diagonal matrix. Skipping this controller configuration');
                end
                it = it+2;
            else
                warning('Missing argument for attitude %s controller. Skipping this controller configuration.',obj.attitudeControllers_{index});
            end
        end
        function it = configureAdaptive(obj,index,varArgs,it)
            if obj.inputsOK(varArgs,it,6)
                it = it+1;
                if isnumeric(varArgs{it}) && all(real(eig(varArgs{it}))<=0)
                    if isnumeric(varArgs{it+1})
                        if isnumeric(varArgs{it+2})
                            if isnumeric(varArgs{it+3})
                                if isnumeric(varArgs{it+4})
                                    if isnumeric(varArgs{it+5})
                                        obj.controlConfig_{index}.Am = varArgs{it};
                                        obj.controlConfig_{index}.Q = varArgs{it+1};                                     
                                        obj.controlConfig_{index}.gamma1 = varArgs{it+2};
                                        obj.controlConfig_{index}.gamma2 = varArgs{it+3};
                                        obj.controlConfig_{index}.gamma3 = varArgs{it+4};
                                        obj.controlConfig_{index}.gamma4 = varArgs{it+5};
                                        
                                        if obj.verbose_ == true
                                            disp([obj.attitudeControllers_{index},' Adaptive reference matrix Am set to: ']);
                                            disp(obj.controlConfig_{index}.Am);
                                            disp([obj.attitudeControllers_{index},' Adaptive matrix Q set to: '])
                                            disp(obj.controlConfig_{index}.Q);   
                                            disp([obj.attitudeControllers_{index},' Adaptive gain Gamma 1 set to: '])
                                            disp(obj.controlConfig_{index}.gamma1);
                                            disp([obj.attitudeControllers_{index},' Adaptive gain Gamma 2 set to: '])
                                            disp(obj.controlConfig_{index}.gamma2);
                                            disp([obj.attitudeControllers_{index},' Adaptive gain Gamma 3 set to: '])
                                            disp(obj.controlConfig_{index}.gamma3);
                                            disp([obj.attitudeControllers_{index},' Adaptive gain Gamma 4 set to: '])
                                            disp(obj.controlConfig_{index}.gamma4);
                                        end
                                    else
                                        warning('Gain Gamma 4 must be a numeric matrix. Skipping this controller configuration');
                                    end
                                else
                                    warning('Gain Gamma 3 must be a numeric matrix. Skipping this controller configuration');
                                end
                            else
                                warning('Gain Gamma 2 must be a numeric matrix. Skipping this controller configuration');
                            end
                        else
                            warning('Gain Gamma 1 must be a numeric matrix. Skipping this controller configuration');
                        end
                    else
                        warning('Q must be a numeric matrix. Skipping this controller configuration');
                    end
                else
                    warning('Reference matrix Am must be a numeric stable matrix. Skipping this controller configuration');
                end
                it = it+5;
            else
                warning('Missing argument for attitude %s controller. Skipping this controller configuration.',obj.attitudeControllers_{index});
            end
        end
        function a = polyMatrix(obj,ti,tf,qi,dqi,d2qi,qf,dqf,d2qf)
            %function [a] = par_pol(to,tf,qo,dqo,d2qo,d3qo,qf,dqf,d2qf,d3qf)
            % Fifth-degree polynomial
            % q  = ao + a1*(t-to) + a2*(t-to)^2  + a3*(t-to)^3 + a4*(t-to)^4 + a5*(t-to)^5;
            
            % q_v = [qo; dqo; d2qo; d3qo; qf; dqf; d2qf; d3qf];
            q_v = [qi; dqi; d2qi; qf; dqf; d2qf];
            delta_t = (tf-ti);
            
            A = [ 1 0 0 0 0 0;
                0 1 0 0 0 0;
                0 0 2 0 0 0;
                1 delta_t delta_t^2 delta_t^3 delta_t^4 delta_t^5;
                0 1     2*delta_t     3*delta_t^2  4*delta_t^3   5*delta_t^4;
                0 0     2             6*delta_t   12*delta_t^2  20*delta_t^3];
            
            a= A\q_v;
        end
        function [ax,ay,az,ayaw] = trajectoryMatrices(obj,to,tf,so,dso,d2so,sf,dsf,d2sf)            
            xo= so(1);
            yo= so(2);
            zo= so(3);
            yawo = so(4);
            xf= sf(1);
            yf= sf(2);
            zf= sf(3);
            yawf = sf(4);
            
            dxo= dso(1);
            dyo= dso(2);
            dzo= dso(3);
            dyawo = dso(4);
            dxf= dsf(1);
            dyf= dsf(2);
            dzf= dsf(3);
            dyawf = dsf(4);
            
            d2xo= d2so(1);
            d2yo= d2so(2);
            d2zo= d2so(3);
            d2yawo = d2so(4);
            d2xf= d2sf(1);
            d2yf= d2sf(2);
            d2zf= d2sf(3);
            d2yawf = d2sf(4);
            
            ax = obj.polyMatrix(to,tf,xo,dxo,d2xo,xf,dxf,d2xf);
            ay = obj.polyMatrix(to,tf,yo,dyo,d2yo,yf,dyf,d2yf);
            az = obj.polyMatrix(to,tf,zo,dzo,d2zo,zf,dzf,d2zf);
            ayaw = obj.polyMatrix(to,tf,yawo,dyawo,d2yawo,yawf,dyawf,d2yawf);
        end
        function [q,dq,d2q,d3q] = axisDesiredTrajectory(obj,a,t,to,tmax)            
            ao = a(1);
            a1 = a(2);
            a2 = a(3);
            a3 = a(4);
            a4 = a(5);
            a5 = a(6);
            
            if t<=to
                delta_t = 0;
            else
                if t>=tmax
                    t = tmax;
                end
                delta_t = t-to;
            end
                
            q  = ao + a1*delta_t + a2*delta_t^2  + a3*delta_t^3 + a4*delta_t^4 + a5*delta_t^5;
            dq = a1 +  2*a2*delta_t + 3*a3*delta_t^2 + 4*a4*delta_t^3 + 5*a5*delta_t^4;
            d2q = 2*a2 + 6*a3*delta_t + 12*a4*delta_t^2 + 20*a5*delta_t^3;
            d3q =  6*a3 + 24*a4*delta_t + 60*a5*delta_t^2;
        end
        function [position, velocity, acceleration, yaw] = desiredTrajectory(obj,time)
            switch obj.trajectoryMap_.type
                case 'waypoints'
                    for index=1:length(obj.trajectoryMap_.endTime)
                        if time <= obj.trajectoryMap_.endTime(index)
                            break;
                        end
                    end
                    ax = obj.trajectoryMap_.ax{index};
                    ay = obj.trajectoryMap_.ay{index};
                    az = obj.trajectoryMap_.az{index};
                    ayaw = obj.trajectoryMap_.ayaw{index};

                    if index==1
                        to = 0;
                    else
                        to = obj.trajectoryMap_.endTime(index-1);
                    end

                    [x,dx,d2x,d3x] = obj.axisDesiredTrajectory(ax,time,to,obj.trajectoryMap_.endTime(index));
                    [y,dy,d2y,d3y] = obj.axisDesiredTrajectory(ay,time,to,obj.trajectoryMap_.endTime(index));
                    [z,dz,d2z,d3z] = obj.axisDesiredTrajectory(az,time,to,obj.trajectoryMap_.endTime(index));
                    [yaw,dyaw,d2yaw,d3yaw] = obj.axisDesiredTrajectory(ayaw,time,to,obj.trajectoryMap_.endTime(index));

                    position = [x;y;z];
                    velocity = [dx;dy;dz];
                    acceleration = [d2x;d2y;d2z];
                case 'gerono'
                    comprimento = obj.trajectoryMap_.length;
                    width = obj.trajectoryMap_.width;
                    height = obj.trajectoryMap_.height;
                    endtime = obj.trajectoryMap_.endTime;
                    yawtype = obj.trajectoryMap_.yawType;
                    t = time;
                    
                    if t>=endtime
                        t = endtime;
                    end
                    
                    a = comprimento/2;
                    b = width;
                    c = height/2;
                    x = -a*cos(2*pi*t/endtime)+a;
                    y = -(b/2)*sin(4*pi*t/endtime);
                    z = c*(1-cos(2*pi*t/endtime));
                    dx = (2*pi*a/endtime)*sin(2*pi*t/endtime);
                    dy = -(2*pi*b/endtime)*cos(4*pi*t/endtime);
                    dz = c*(2*pi/endtime)*sin(2*pi*t/endtime);
                    d2x = a*(2*pi/endtime)^2*cos(2*pi*t/endtime);
                    d2y = (8*pi*pi*b/(endtime^2))*sin(4*pi*t/endtime);
                    d2z = c*(2*pi/endtime)^2*cos(2*pi*t/endtime);

                    switch yawtype
                        case 'fixed'
                            yawAngle = obj.trajectoryMap_.yawAngle;
                            yaw = yawAngle;
                            dyaw = 0;
                            d2yaw = 0;
                        case '360'
                            yaw = pi*(1-cos(pi*t/endtime));
                            dyaw = pi*(pi/endtime)*sin(pi*t/endtime);
                            d2yaw = pi*(pi/endtime)^2*cos(pi*t/endtime);
                        case 'goto'
                            yawAngle = obj.trajectoryMap_.yawAngle;
                            yaw = yawAngle*(1-cos(pi*t/endtime))/2;
                            dyaw = yawAngle*(pi/endtime)*sin(pi*t/endtime)/2;
                            d2yaw = yawAngle*(pi/endtime)^2*cos(pi*t/endtime)/2;
                        case 'sinusoidal'
                            mean = obj.trajectoryMap_.yawMean;
                            max = obj.trajectoryMap_.yawMax;
                            period = obj.trajectoryMap_.yawPeriod;
                            yaw = max*sin(2*pi*t/period)+mean;
                            dyaw = max*(2*pi/period)*cos(2*pi*t/period);
                            d2yaw = -max*(2*pi/period)^2*sin(2*pi*t/period);     
                    end
                    position = [x;y;z];
                    velocity = [dx;dy;dz];
                    acceleration = [d2x;d2y;d2z];
            end
        end
        function angle =  normalizeAngle(obj, r,low)
        % Compute the angle in the semi-closed interval [low, low + 2PI) that represents the same rotation as the angle <em>r</em>
        % r   angle in radians
        % low starting value of the normalization interval
        % angle = equivalent angle in the interval [low, low + 2PI) computed as r - 2PI * FLOOR((r - low) / (2PI)) 

            angle = r - 2*pi*floor((r - low)/(2*pi));   
        end
        function [corpo] = moveObjectDisplay(obj,corpo,att,pos)
            % Move objeto
            % [corpo] = move_objeto_f(corpo,att,pos)
            R_x = [1 0 0;
                   0 cos(att(1)) -sin(att(1));
                   0 sin(att(1))  cos(att(1))];
               
            R_y = [cos(att(2))  0 sin(att(2));
                   0         1              0;
                   -sin(att(2)) 0 cos(att(2))];
            R_z = [cos(att(3))  -sin(att(3)) 0;
                   sin(att(3))   cos(att(3)) 0;
                   0          0        1];
            R = R_z*R_y*R_x;

            for cont = 1:corpo.n
                corpo.vertices(cont,:) = (R*corpo.vertices(cont,:)' + pos)';    
            end

            set(corpo.objeto,'vertices',corpo.vertices);
            drawnow
        end
        function [result,successTime] = verifyMissionSuccess(obj)
            successTime = inf;
            result = false;
            checkedWaypoints = zeros(1,length(obj.trajectoryMap_.endTime));
            counter = 1;
            for it=1:length(obj.trajectoryMap_.endTime)
                [position, ~, ~, yaw] = obj.desiredTrajectory(obj.trajectoryMap_.endTime(it));
                while counter<=size(obj.log_.position,2) && checkedWaypoints(it)==0
                    if obj.distance(position,obj.log_.position(:,counter))<=obj.metrics_.missionMetricPrecision && ...
                            obj.distanceAng(obj.log_.attitude(:,counter),[cos(yaw/2) 0 0 sin(yaw/2)]')<=(pi/180)*obj.metrics_.missionAngularPrecision
                        checkedWaypoints(it) = 1;
                        result = all(checkedWaypoints);
                        if it == length(obj.trajectoryMap_.endTime) && result == true
                            successTime = obj.log_.time(counter);
                        end
                    end
                    counter = counter+1;
                end
            end
        end
        function distanceResult = distance(obj, point1, point2)
            distanceResult = sqrt((point1(1)-point2(1))^2+(point1(2)-point2(2))^2+(point1(3)-point2(3))^2);
        end
        function distanceResult = distanceAng(obj,q,qd)
            % Part of the quaternion error used in the attitude controller
            qe1 = + qd(1)*q(1) + qd(2)*q(2) + qd(3)*q(3) + qd(4)*q(4);
            distanceResult = 2*acos(qe1);%in radians
        end
        function clearTrajectory(obj)
            obj.trajectory_.position        = [];      %3xN Matrix
            obj.trajectory_.velocity        = [];      %3xN Matrix
            obj.trajectory_.acceleration    = [];  %3xN Matrix
            obj.trajectory_.attitude        = [];      %4xN Matrix (quaternions)
            obj.trajectory_.angularVelocity = [];      %3xN Matrix (euler)
            obj.trajectory_.time            = [];          %1xN Matrix
        end
        function clearMetrics(obj)
            obj.metrics_.simulationSuccess      = [];
            obj.metrics_.simulationEndError     = [];
            obj.metrics_.missionSuccess         = [];
            obj.metrics_.missionMetricPrecision = [];
            obj.metrics_.missionAngularPrecision= [];
            obj.metrics_.isStable               = [];
            obj.metrics_.pathLength             = [];
            obj.metrics_.desiredPathLength      = [];
            obj.metrics_.pathLengthRatio        = [];
            obj.metrics_.pathTimeRatio          = [];
            obj.metrics_.averageSpeed           = [];
            obj.metrics_.maxSpeed               = [];
            obj.metrics_.meanPositionError      = [];
            obj.metrics_.RMSPositionError       = [];
            obj.metrics_.maxPositionError       = [];
            obj.metrics_.meanAngularError       = [];
            obj.metrics_.RMSAngularError        = [];
            obj.metrics_.maxAngularError        = [];
            obj.metrics_.energy                 = [];
            obj.metrics_.RMSPower               = [];
            obj.metrics_.maxPower               = [];
        end
        function clearFDD(obj)
            for it=1:obj.numberOfRotors_ 
                obj.fddRotor_{it}.status = {};
                obj.fddRotor_{it}.status{1} = {'free','responding','motor ok','prop ok'};
                obj.fddRotor_{it}.motorEfficiency = 1.0;
                obj.fddRotor_{it}.propEfficiency = 1.0;            
                obj.fddRotor_{it}.time = 0;
                obj.roulette = [];
            end
        end
    end
end

