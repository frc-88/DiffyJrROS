M_inv = [
    14, 6;
    -10, 6
];
J_w = 0.015;

Kt = 0.01825;
Kv = 56.0;
R = 0.0467;

A = -Kt / (Kv * R * J_w) * M_inv * M_inv;
A = [1, 0; A];
A_column_0 = zeros([3, 1]);
A = [A_column_0, A];

B = [0, 0;
    M_inv * Kt / (R * J_w)];

C = eye(3);
D = zeros([3, 2]);

Q = [0.08, 0, 0;
     0, 1.1, 0;
     0, 0, 1.0];

R = [12, 0;
     0, 12];

Q_model = [
    0.1, 0, 0;
    0, 5.0, 0;
    0, 0, 5.0
];
R_model = [
    0.5, 0, 0;
    0, 0.1, 0;
    0, 0, 0.1
];

[LQR_K] = lqr(A, B, Q, R);

out = sim("DiffSwerveModule.slx", 10);
% sim_result = reshape(out.sim_response.Data, [], 1);
% sim_time = out.sim_response.Time;
