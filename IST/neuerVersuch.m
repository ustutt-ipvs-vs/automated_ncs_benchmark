%% Parameters
% System parameter
% g = 9.81;
% kappa = 1;
% lp = 0.3302;
% 
% %% Continuous system
% A_cont=[0,   0,1,0;
%         0,   0,0,1;
%         0,   0,0,0;
%         0,g/lp,0,0];
% 
% B_cont=[0;0;kappa;kappa/lp];
% Bw_cont=[0;0;kappa;kappa/lp];
% 
% C_cont=[1,0,0,0;0,1,0,0];
% Cz_cont=[1,0,0,0;0,1,0,0];
% 
% Dz_cont=[0;0];



%% Computation of worst delay

h = 10e-3; % smallest sampling stepsize
% delayPrios = [2; 4; 6; 8; 10; 12; 14]e-3;
basePeriod = 45e-3; % seconds
% 
% lowestPrio = 4;
maxSamplingStepsize = 100e-3;

lowestUsedPrioDelayInS = 8e-3;

maxDelayPrios = lowestUsedPrioDelayInS + max(basePeriod,maxSamplingStepsize);
delay = ceil(maxDelayPrios/h)


%% Aus Julia:
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

% % TODO alternative model from PERT?
% kappa = eta_g*eta_m*K_g*K_t/(R_m*r_mp*(m_p*m));
% A_cont = [0   1   0   0;
%      0   0   0   0;
%      0   0   0   1;
%      0   0  g/lp  0]
% B_cont = [0; kappa; 0; kappa/lp]

Bw_cont=[0;1;0;1];
% Cz_cont=[1,0,0,0; 0,1,0,0; 0,0,1,0];
Cz_cont=[1,0,0,0; 0,1,0,0; 0,0,1,0; 0,0,0,1];
% C_cont=[1,0,0,0; 0,1,0,0; 0,0,1,0];
C_cont=[1,0,0,0; 0,1,0,0; 0,0,1,0; 0,0,0,1];
% TODO error if C_cont ~= CZ_cont
% Dz_cont=[0;0;0];
% Dz_cont=[0;0;0;0];
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

% delay=2-1;

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

%poleFactor = 1/0.9;
%con = blkdiag(con,[X, poleFactor*A*X + poleFactor*B*Z; (poleFactor*A*X + poleFactor*B*Z)', X]>=eps)

s = optimize(con, gamma, opt);
gamma_iqc=sqrt(value(gamma))

% Minimize controller
con=[[blkdiag(X,X,gamma_iqc^2*(1/0.9)*eye(mw)), sysda';
      sysda, blkdiag(X,(1/(delay-1))*X,eye(n_w))]>=eps];
s = optimize(con, norm(Z), opt);

K_iqc=value(Z)*(inv(value(X)))

disp("Hinweis zur Ausgabe: Erster Wert von K_iqc (bzw. K_ss) ist für Kxic (integral feedback). Restliche 4 Werte sind Kxc, Kvc, Kxp, Kvp")
%
%% Switched system approach
%

A_lift=eye(n_s);
Bw_lift=[];
Cz_lift=[];
B_lift=0;
D_lift=D;

clear X
G=sdpvar(n_s,n_s,'full');
Z=sdpvar(m,n_s,'full');
for i=1:1:delay+1
    X{i}=sdpvar(n_s);
end

con=[];

for i=1:1:delay+1
    A_lift=A_lift*A;
    B_lift=B_lift+A^(i-1)*B;

    Bw_lift=[A^(i-1)*B,Bw_lift];
    Cz_lift=[Cz_lift;Cz*A^(i-1)];

    Dw_lift=0;
    Dw_lift = Dw_lift + kron(diag(1:1:i),D);

    for j=1:1:i-1
        Dw_lift = Dw_lift + kron(diag(ones(1,i-j),-j),C*A^(j-1)*B);
%         Dw_lift = Dw_lift + kron(A^(j-1),diag(ones(1,h-j),-j));
    end

    if i ~= 1
        D_lift = [D_lift; D_lift(max(end-n_w+1,1):end,:) + Cz*A^(i-2)*B];
    end

    for j=1:1:delay+1
        ineq=[G+G'-X{i}   , Z'*B_lift'+G'*A_lift' , G'*Cz_lift'+Z'*D_lift';
              B_lift*Z+A_lift*G ,  X{j} - Bw_lift*gamma*Bw_lift', -Bw_lift*gamma*Dw_lift';
              Cz_lift*G+D_lift*Z , -Dw_lift*gamma*Bw_lift', eye(i*n_w)-Dw_lift*gamma*Dw_lift'];

        con=[con, ineq>=eps];
    end


end


s = optimize(con, -gamma, opt);
gamma_ss=sqrt(1/value(gamma))
K_ss=value(Z)*inv(value(G))


% %% System for synthesis
% m=size(B_disc,2);
% mw=size(Bw_disc,2);
% 
% n_w=size(Cz_disc,1);
% 
% A=A_disc;
% A_c=B;
% 
% B=[zeros(n,m),Bw_disc];
% B_c=[B_disc, zeros(n, mw)];
% 
% C=[eye(n)-A_disc;C_disc];
% C_c=[-B;B];
% 
% D=[zeros(n_s, m+mw);zeros(n_w,m),Dz_disc];
% D_c=[B_disc;zeros(n_s,mw);zeros(n_w,m+w)];
% 
% %% LMI
% 
%     ineq=[X   , Z'*A_c'+X*A' , X*C'+Z'*C_c';
%           A_c*Zi+A*X'  ,  X- Bucw{l}*gamma*Bucw{l}', -Bucw{l}*gamma*Ducw{l}';
%           Cuc{l}*Gi'+Ducu{l}*Zi  , -Ducw{l}*gamma*Bucw{l}', eye(n)-Ducw{l}*gamma*Ducw{l}'];