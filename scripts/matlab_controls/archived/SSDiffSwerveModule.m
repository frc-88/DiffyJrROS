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

module_ss = ss(A, B, C, D);

Q = [0.08, 0, 0;
     0, 1.1, 0;
     0, 0, 1.0];
%Q = Q * 100;
R = [12, 0;
     0, 12];
[K] = lqr(A, B, Q, R);

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

% [kalmf,L,P] = kalman(module_ss, Q, R);


Nbar = rscale(A, B, C, D, K);
Bbar_1 = B(:, 1) * Nbar(1);
Bbar_2 = B(:, 2) * Nbar(2);
Bbar = [
    Bbar_1(:), Bbar_2(:)
];
sys_cl = ss(A - B * K, Bbar, C, D);

t=0:0.1:10;

% opt = stepDataOptions('InputOffset', 0.0, 'StepAmplitude', [1.0, 1.0]);
% step(sys_cl, opt);
% step(module_ss, opt);

% u = [zeros(size(t)); ones(size(t)) * 12.0];
u = [ones(size(t)) * 12; ones(size(t)) * 12];
[Y, T] = lsim(sys_cl, u, t);
close all
hold all
figure(1)
plot(T, Y)
legend('angle', 'angular v', 'wheel v')
