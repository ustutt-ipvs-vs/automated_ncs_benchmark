format long g % Don't print numbers in scientific notation

l = 0.6;
g = 9.8088;
mass = 1.0;
d = l/2.0;
I = l^2/3.0;

Ts = 0.05;
delay = 0.005;

% x = (cartPos, cartSpeed, poleAngle, poleSpeed)
K_param = mass * d / (mass * d^2 + I);
A = [0   1   0   0;
     0   0   0   0;
     0   0   0   1;
     0   0  K_param*g  0];
B = [0; 1; 0; K_param];

Q = [1,0,0,0;
    0,0.01,0,0;
    0,0,1.5,0;
    0,0,0,0.01];
R = 0.03;

n = 4; % Dimension of state vector x_k
m = 1; % Dimension of input vector u_k
h = Ts; % Sampling period in seconds
tau_k = delay; % delay in seconds

% Calculate phi, Gamma_1 and Gamma_0
phi = expm(A*h);
Gamma_1 = integral(@(s)expm(A*s), h-tau_k, h, 'ArrayValued', true) * B;
Gamma_0 = integral(@(s)expm(A*s), 0, h-tau_k, 'ArrayValued', true) * B;


% Calculate parameter matrices under consideration of delays:
A_z = [phi, Gamma_1; zeros(m, n), zeros(m, m)];
B_z = [Gamma_0; eye(m)];
Q_z = blkdiag(Q, R);
R_z = R;

% Create extended state space model:
C_z = eye(n+m);
D_z = zeros(n+m, m);

% Convert continuous state space model into a discrete model with sampling
% period Ts:
sys_cont = ss(A_z, B_z, C_z, D_z);
sys_disc = c2d(sys_cont, Ts);

% Calculate LQR for the discrete system:
[K_z, S_z, P_z] = lqr(sys_disc, Q_z, R_z);

K_z_neg = - K_z
