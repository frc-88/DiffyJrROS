%% Inverted Pendulum with Animation
m = 0.21;  % mass of the pendulum (kg)
Mcart = 0.455;  % mass of the cart (kg)
l = 0.61 / 2;  % distance to pendulum center of mass (m)
g = 9.8;  % acceleration due to gravity (m/s^2)

Ts = 0.02;  % discrete time step width

A = zeros(2);
B = -eye(2)/Ts;
C = [0 0; 1 0; 0 0; 0 1];
D = [1 0; 1/Ts 0; 0 1; 0 1/Ts];

% Q = [0.1, 0;
%      0, 0.1];
% R = [0.01, 0;
%      0, 0.01];
% K = lqr(A, B, Q, R);
% K = reshape(K, 1, []);
K = [0 -18 -166.5 -15.2];

open_system('PendulumDynamics');
set_param('PendulumDynamics', 'StopTime', 'inf');
sim('PendulumDynamics');

