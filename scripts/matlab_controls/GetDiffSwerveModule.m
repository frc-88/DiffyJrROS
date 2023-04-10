function [module] = GetDiffSwerveModule()
M_inv = [
    14, 6;
    -10, 6
];
J_w = 0.015;

D_w = 0.0445;

Kt = 0.01825;
Kv = 56.0;
Res = 0.0467 / 2;
Vmax = 12.0;
Imax = min(60.0, Vmax / Res);
% Vmax = min(Vmax, Imax * Res);

A = -Kt / (Kv * Res * J_w) * M_inv * M_inv;
A = [1, 0; A];
A_column_0 = zeros([3, 1]);
A = [A_column_0, A];

B = [0, 0;
    M_inv * Kt / (Res * J_w)];

C = eye(3);
D = zeros([3, 2]);

Ts = 0.005;

sysc = ss(A, B, C, D);
sysd = c2d(sysc, Ts);

Q = [1.0, 0, 0;
     0, 10.0, 0;
     0, 0, 10.0];

R = [12, 0;
     0, 12];

Q = eye(3) / Q^2;
R = eye(2) / R^2;

lqr_K = lqr(sysd, Q, R);
Kff = pinv(sysd.B);

module = struct(...
    'M_inv', M_inv,...  % inverse kinematics for diff swerve gear box
    'J_w', J_w,...  % moment of inertia of the wheel
    'Kt', Kt,...  % Kt motor property
    'Kv', Kv,...  % Kv motor property
    'Res', Res,...  % motor resistance
    'sysc', sysc,...  % continuous state space system object
    'sysd', sysd,...  % discrete state space system object
    'lqr_K', lqr_K,...  % LQR gain
    'Kff', Kff,...  % feedforward gain
    'D_w', D_w,...  % wheel diameter
    'Ts', Ts,...  % discrete time sample size)
    'Vmax', Vmax,...  % max voltage allowed to the motors
    'Imax', Imax...  % max current allowed to the motors
);
end

