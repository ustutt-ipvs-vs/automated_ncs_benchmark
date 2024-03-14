
l = 0.6;
g = 9.8088;
m = 1.0;
d = l/2.0;
I = l^2/3.0;

Ts = 0.05;
delay = 0.01;

% x = (cartPos, cartSpeed, poleAngle, poleSpeed)
K_param = m * d / (m * d^2 + I);
A_cont = [0   1   0   0;
     0   0   0   0;
     0   0   0   1;
     0   0  K_param*g  0];
B_cont = [0; 1; 0; K_param];
C_cont=[1,0,0,0; 0,1,0,0; 0,0,1,0; 0,0,0,1];

%sys_cont = ss(A_cont,B_cont,C_cont,0*C_cont*B_cont,'OutputDelay',[0,0,delay,0]);
sys_cont = ss(A_cont,B_cont,C_cont,0*C_cont*B_cont);
sys_disc = c2d(sys_cont,Ts);
A = sys_disc.A;
B = sys_disc.B;
C = sys_disc.C;
D = sys_disc.D;

Q = [1,0,0,0;
    0,0.01,0,0;
    0,0,1.5,0;
    0,0,0,0.01];
R = 0.03;

%[K,S,P] = dlqr(A,B,Q,R);
[K,S,P] = lqr(sys_disc,Q,R)

K = -K