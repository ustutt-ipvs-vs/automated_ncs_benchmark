% Konstanten (sollte Matlab-Skript berechnen): 

% - A € R^{4x4} (in Wiki F)
% - B € R^{4x1}
% A und B hängen von der Samplingperiode h ab. (siehe neuerVersuch.m % Discrete system)
% A=eye(n_s)+A_cont*h;
% B=B_cont*h;
% Das müsste man vorberechnen und im Arduino mit switch-case der samplingPeriod anpassen.
% Oder man berechnet das aktuelle A und B online. Es ist ja ein einfacher
% Zusammenhang zu h. Dann müsste man nur A_cont und B_cont speichern.

% - C (in Wiki H) bleibt konstant (wie in demonstrator.jl)
 % 1.0  0.0  0.0  0.0
 % 0.0  1.0  0.0  0.0
 % 0.0  0.0  1.0  0.0
 % wenn (cartPos, cartSpeed, poleAngle) gemessen wird. Wenn wir zusätzlich
 % poleSpeed messen können, haben wir einfach I (4x4).
% - Q (4x4) skaliert mit h, also Q = Q0*h mit konstantem Parameter Q0, den
% wir zum tuning verwenden können.
% - R (1x1) skaliert mit h, also R = R0*h mit konstantem Parameter R0, den
% wir zum tuning verwenden können.
% - P_{0|-1} = sigma^2 * I (4x4)
% - x_{0|-1} = [0; 0; 0; 0]; Vektor mit Anfangszustand. 
 
% Messung:

% - z_k € R^{3x1} (cartPos, cartSpeed, poleAngle)
 % z_k ist ein Vektor und so groß wie die Anzahl der Messwerte, die wir in
 % jedem Schritt haben. In dem Fall 3. Wenn wir zusätzlich poleSpeed messen
 % können, haben wir (4x1).

% Werte aus Schritt k-1:

% - x_{k-1} € R^{4x1}
% - u_{k-1} % Ist das die Ausgabe des LQR im Schritt k-1 (heißt im Code u_accel)?
 % Genau richtig. Das ist der input, den wir im letzten Schritt ins System
 % gegeben haben. Das brauchen wir, um nachher in Schritt 1. mit dem Modell den Zustand zu
 % aktualisieren.
% - P_{k-1} (4x4)

% Berechnen im Schritt k:

% 1. x_{k|k-1} = A * x_{k-1} + B * u_{k-1} % State Prediction
% 2. P_{k|k-1} = A * P_{k-1} * A^T + Q % State Covariance Prediction
% 3. K_k = P_{k|k-1} * C^T * (C * P_{k|k-1} * C^T + R)^{-1} % Kalman Gain
% 4. x_k = x_{k|k-1} + K_k * (z_k - C * x_{k|k-1}) % State Update
% 5. P_k = (I - K_k * C) * P_{k|k-1} % State Covariance Update

% Sollte alles so stimmen.

%% Hier hab ich mal noch eine implementierung aus einem älteren Projet angehängt. Vielleicht kannst du dich inspirieren lassen.

%% Initialization

x_0 = [0; 0; 0; 0];

%% model
% state vector x = [cartPos, cartSpeed, poleAngle, poleSpeed]'
% x_k+1 = F_cv * x_k + G_cv * w_k
% y = H*x_k + e_k

F_2 = [1 T; 0 1];
G_2 = eye(2);

F_ev = kron(eye(2),F_2);
G_ev = kron(eye(2),G_2);

% state transition matrix
F = F_ev;
%
G = G_ev;

% output matrix
H=zeros(2,4);
H(1,1)=1;
H(2,3)=1;


%% Initialization for Kalman filter
XoverTime = zeros(4,N);
PoverTime = zeros(4,4,N);

% initial state
% X=zeros(4,1);
X = x_0;

% initial state covariance
P=zeros(4);

X_k_km1 = X;
P_k_km1 = P;


%% Filtering over all time steps
for t=1:N
        
    R1 = eye(4)*0.1;
    R2 = eye(2)*5*100;
    
    % get the measurement
    Y=resA.measurement(:,t);
    
    % Measurement prediction
    Y_eq = H*X_k_km1;
    % Kalman Gain
    K = P_k_km1*H'/(H*P_k_km1*H' + R2);
    % State Update
    X = X_k_km1 + K*(Y-Y_eq);
    % State Covariance Update
    P = P_k_km1 - K*H*P_k_km1;
    
    XoverTime(:,t) = X;
    PoverTime(:,:,t) = P;
    
    % State Prediction
    X_k_km1 = F*X;
    % State Covariance Prediction
    P_k_km1 = F*P*F' + G*R1*G';
    
    
end
