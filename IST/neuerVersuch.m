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


%% Aus Julia:

l = 0.6;
g = 9.8088;
sys.m = 1.0;
sys.d = l/2.0;
sys.I = l^2/3.0;

K = sys.m * sys.d / (sys.m * sys.d^2 + sys.I)
A_cond = [0   1   0   0;
     0   0   0   0;
     0   0   0   1;
     0   0  K*g  0]
B_cond = [0; 1; 0; K]

Bw_cont=[0;0;0;1];
Cz_cont=[1,0,0,0;0,1,0,0];
C_cont=[1,0,0,0;0,1,0,0];



%% Discrete system
h = 0.05;
n_s= size(A_cont,1);

A=eye(n_s)+A_cont*h;

B=B_cont*h;
Bw=Bw_cont*h;

C = C_cont;
Cz = Cz_cont;

Dz=Dz_cont;
D=0*C*B;

%% LMI
opt = sdpsettings('verbose', 0, 'solver', 'mosek');
eps = 1e-8;

delay=2-1;

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
      sysda, blkdiag(X,(1/delay)*X,eye(n_w))]>=eps];

s = optimize(con, gamma, opt);
gamma_iqc=sqrt(value(gamma))
K_iqc=value(Z)*(inv(value(X)))


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
    Dw_lift = Dw_lift + kron(D,diag(1:1:i));

    for j=1:1:delay-1
        Dw_lift = Dw_lift + kron(A^(j-1),diag(ones(1,h-j),-j));
    end

    if i ~= 1
        D_lift = [D_lift; D_lift(end:-1:max(end-n_w,1),:) + Cz*A^(i-2)*B];
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