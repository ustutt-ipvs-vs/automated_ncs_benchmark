

%% Computation of worst delay

h = 20e-3; % smallest sampling period used

basePeriod = 40e-3; % bucket refill rate r

maxSamplingStepsize = 200e-3; % largest sampling period used

lowestUsedPrioDelayInS = 14e-3;

maxDelayPrios = lowestUsedPrioDelayInS + max(basePeriod,maxSamplingStepsize);
delay = ceil(maxDelayPrios/h)


%% Pendulum parameters:
l = 0.6;
g = 9.8088;
sys.m = 1.0;
sys.d = l/2.0;
sys.I = l^2/3.0;

% x = (cartPos, cartSpeed, poleAngle, poleSpeed)
K_param = sys.m * sys.d / (sys.m * sys.d^2 + sys.I)
A_cont = [0   1   0   0;
     0   0   0   0;
     0   0   0   1;
     0   0  K_param*g  0]
B_cont = [0; 1; 0; K_param]


Bw_cont=[0;1;0;1];

Cz_cont=[1,0,0,0; 0,1,0,0; 0,0,1,0; 0,0,0,1];

C_cont=[1,0,0,0; 0,1,0,0; 0,0,1,0; 0,0,0,1];

Dz_cont=zeros(size(Cz_cont,1),1);

% with integrator
% x = (cartPosInt, cartPos, cartSpeed, poleAngle, poleSpeed)
A_cont = [0 1 0 0 0;
          zeros(4,1) A_cont]
B_cont = [0;B_cont];
Bw_cont=[0;Bw_cont];
Cz_cont=[1 0 0 0 0;
        zeros(size(Cz_cont,1),1) Cz_cont];
C_cont=[1 0 0 0 0;
        zeros(size(C_cont,1),1) C_cont];
Dz_cont=[0;Dz_cont];


%% Discrete system
n_s= size(A_cont,1);

A=eye(n_s)+A_cont*h;

B=B_cont*h;
Bw=Bw_cont*h;

C = C_cont;
Cz = Cz_cont;

Dz=Dz_cont;
D=0*C*B;

%% Matlab discretization
sys_cont = ss(A_cont,Bw_cont,Cz_cont,Dz_cont);
sys_disc = c2d(sys_cont,h);
A = sys_disc.A;
Bw = sys_disc.B;
Cz = sys_disc.C;
Dz = sys_disc.D;

sys_cont = ss(A_cont,B_cont,C_cont,0*C_cont*B_cont);
sys_disc = c2d(sys_cont,h);
B = sys_disc.B;
C = sys_disc.C;
D = sys_disc.D;

%% LMI
opt = sdpsettings('verbose', 0, 'solver', 'mosek');
eps = 1e-8;

m=size(B,2);
mw=size(Bw,2);

n_w=size(Cz,1);

X=sdpvar(n_s);
Z=sdpvar(m,n_s,'full');
gamma= sdpvar(1);


sysda = [A*X+B*Z  , B*Z           , Bw;
         X-A*X-B*Z, -B*Z          , zeros(n_s,mw);
         Cz*X    , zeros(n_w,n_s),Dz];


con=[[blkdiag(X,X,gamma*eye(mw)), sysda';
      sysda, blkdiag(X,(1/(delay-1))*X,eye(n_w))]>=eps];

s = optimize(con, gamma, opt);
gamma_iqc=sqrt(value(gamma))

% Minimize controller
con=[[blkdiag(X,X,gamma_iqc^2*(1/0.9)*eye(mw)), sysda';
      sysda, blkdiag(X,(1/(delay-1))*X,eye(n_w))]>=eps];
s = optimize(con, norm(Z), opt);

K_iqc=value(Z)*(inv(value(X)))

disp("How to interpret the output: First value of K_iqc is the integral feedback (Kxic), called 'controllerIntegratorParam' in the config file. The following 4 values are Kxc, Kvc, Kxp, Kvp and used in this order for 'controllerKVector'.")
