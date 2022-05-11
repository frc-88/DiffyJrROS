classdef DiffSwerveChassis < matlab.System
    %DIFFSWERVECHASSIS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        modules
        M_inv  % chassis inverse kinematics
        M  % chassis forward kinematics
        num_modules
        module_locations  % module placement in meters. Ex: [0, 0; 1, 1; 2, 2; 3, 3]
    end
    
    methods
        function obj = DiffSwerveChassis(module_locations)
            %DIFFSWERVECHASSIS Construct an instance of this class
            %   Detailed explanation goes here
            num_modules = size(module_locations, 1);
            modules(1:num_modules) = DiffSwerveModule;

            obj.modules = modules;

            M_inv = zeros(num_modules * 2, 3);
            for index = 0:num_modules - 1
                M_inv(index * 2 + 1, :) = [1, 0, -module_locations(index + 1, 2)];
                M_inv(index * 2 + 2, :) = [0, 1, +module_locations(index + 1, 1)];
            end
            M = pinv(M_inv);

            obj.M_inv = M_inv;
            obj.M = M;
            obj.num_modules = num_modules;
            obj.module_locations = module_locations;

        end
        
        function chassisState = forwardKinematics(obj, moduleStates)
            %FORWARDKINEMATICS run forward kinematics for swerve chasssis
            %   moduleStates is an n by 3 matrix where n in the number of
            %   modules. column 1 is azimuth, 2 is azimuth velocity, 3 is
            %   wheel velocity
            
            module_state_matrix = zeros(obj.num_modules * 2, 1);
            for index = 0:obj.num_modules - 1
                azimuth = moduleStates(1);
                % index 2 is azimuth velocity
                wheel_velocity = moduleStates(3);
                module_state_matrix(index * 2 + 1) = wheel_velocity * cos(azimuth);
                module_state_matrix(index * 2 + 2) = wheel_velocity * sin(azimuth);
            end
            chassisState = obj.M * module_state_matrix;
        end

        function moduleStates = inverseKinematics(obj, chassisState)
            %INVERSEKINEMATICS run inverse kinematics for swerve chassis
            %   chassisState is a 1 by 3 matrix where 1 is vx, 2 is vy, 3
            %   is vt. Units are meters per second and radians per second
            vector_states = obj.M_inv * chassisState';
            moduleStates = zeros(obj.num_modules, 3);

            for index = 0:obj.num_modules - 1
                vx = vector_states(index * 2 + 1);
                vy = vector_states(index * 2 + 2);
                wheel_speed = hypot(vx, vy);
                azimuth = atan2(vy, vx);
                moduleStates(index + 1, 1) = azimuth;
                % azimuth velocity is not calculated
                moduleStates(index + 1, 3) = wheel_speed;
            end
        end
    end
end

