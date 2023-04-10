function [chassis] = GetDiffSwerveChassis(module_locations)
num_modules = size(module_locations, 1);

M_inv = zeros(num_modules * 2, 3);
for index = 0:num_modules - 1
    M_inv(index * 2 + 1, :) = [1, 0, -module_locations(index + 1, 2)];
    M_inv(index * 2 + 2, :) = [0, 1, +module_locations(index + 1, 1)];
end
M = pinv(M_inv);

chassis = struct(...
    'M_inv', M_inv,...
    'M', M,...
    'num_modules', num_modules,...
    'module_locations', module_locations...
);
end

