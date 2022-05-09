m = 2.2675;  % Mass of arm in kg
l = 1.2192;  % Length of arm in m
J = 1 / 3 * m * l^2;  % Arm moment of inertia in kg-m^2
G = 1.0 / 2.0;  % Gear ratio

max_V = 12;  % maximum magnitude of voltage

lqr_Q = [0.01745, 0.08726];

kalman_Q = [0.01745, 0.1745329];
kalman_R = [0.01, 0.01];

% motor constants
Kt = 0.01824902723735409;
Kv = 56.00286812769124;
R = 0.023346303501945526;

A = [0, 1;
    0, -G^2 * Kt / (Kv * R * J)];
B = [0;
    G * Kt / (R * J)];
C = eye(2);
D = [0; 0];

poles = [-8 + 11i, -8 - 11i];
K = place(A, B, poles);

sim("SingleJointedArm.slx")
