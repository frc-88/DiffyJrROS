M_inv = [
    14, 6;
    -10, 6
];
J_w = 0.015;

Kt = 0.01825;
Kv = 56.0;
R = 0.0467 / 2;

A = -Kt / (Kv * R * J_w) * M_inv * M_inv;
A = [1, 0; A];
A_column_0 = zeros([3, 1]);
A = [A_column_0, A];

B = [0, 0;
    M_inv * Kt / (R * J_w)];

C = eye(3);
D = zeros([3, 2]);

Ts = 0.005;

sysc = ss(A, B, C, D);
sysd = c2d(sysc, Ts);

% Better setpoint tracking
% Q = [0.04, 0, 0;
%      0, 1.0, 0;
%      0, 0, 1.5];
% Less saturation
Q = [1.0, 0, 0;
     0, 10.0, 0;
     0, 0, 10.0];

R = [12, 0;
     0, 12];

Q = eye(3) / Q^2;
R = eye(2) / R^2;
lqr_K = lqr(sysd, Q, R);
% P = idare(sysd.A, sysd.B, Q, R, [], []);
% lqr_K = linsolve(R + sysd.B' * P * sysd.B, sysd.B' * P * sysd.A);

Kff = pinv(sysd.B);

% simulationName = "ContinuousDiffSwerveModuleSimulation";
% simulationName = "DiscreteDiffSwerveModuleSimulation";
simulationName = "DiffSwerveChassis";

open_system(simulationName);
set_param(simulationName, 'StopTime', '2.0');
sim(simulationName)
