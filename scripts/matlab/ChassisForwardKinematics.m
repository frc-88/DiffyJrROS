function [chassisState] = ChassisForwardKinematics(M, moduleStates)
%FORWARDKINEMATICS run forward kinematics for swerve chasssis
%   moduleStates is an n by 3 matrix where n in the number of
%   modules. column 1 is azimuth, 2 is azimuth velocity, 3 is
%   wheel velocity

num_modules = size(moduleStates, 1);

module_state_matrix = zeros(num_modules * 2, 1);
for index = 0:num_modules - 1
    azimuth = moduleStates(index + 1, 1);
    % index 2 is azimuth velocity
    wheel_velocity = moduleStates(index + 1, 3);
    module_state_matrix(index * 2 + 1) = wheel_velocity * cos(azimuth);
    module_state_matrix(index * 2 + 2) = wheel_velocity * sin(azimuth);
end
chassisState = M * module_state_matrix;
end

