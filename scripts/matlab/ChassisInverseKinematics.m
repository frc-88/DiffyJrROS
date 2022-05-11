function [moduleStates] = ChassisInverseKinematics(M_inv, num_modules, chassisState)
%INVERSEKINEMATICS run inverse kinematics for swerve chassis
%   chassisState is a 1 by 3 matrix where 1 is vx, 2 is vy, 3
%   is vt. Units are meters per second and radians per second
vector_states = M_inv * chassisState;
moduleStates = zeros(num_modules, 3);

for index = 0:num_modules - 1
    vx = vector_states(index * 2 + 1);
    vy = vector_states(index * 2 + 2);
    wheel_speed = hypot(vx, vy);
    azimuth = atan2(vy, vx);
    moduleStates(index + 1, 1) = azimuth;
    % azimuth velocity is not calculated
    moduleStates(index + 1, 3) = wheel_speed;
end
end

