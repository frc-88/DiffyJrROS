width = 0.30861;
length = 0.30861;
locations = [
    width / 2, length / 2;
    -width / 2, length / 2;
    -width / 2, -length / 2;
    width / 2, -length / 2;
];

chassis = GetDiffSwerveChassis(locations);
module = GetDiffSwerveModule;

% simulationName = "DiffSwerveSystemSimulation";
simulationName = "CurrentTestSimulation";

x0 = -1.0;
y0 = 2.0;
theta0 = 0.0;

open_system(simulationName);
set_param(simulationName, 'StopTime', '10.0');
out = sim(simulationName);

% hold on
% plot(out.position.data(:, 1), out.position.data(:, 2))
% axis equal
