# LQR-Regler mit Delay: Aufschrieb
Dieser Aufschrieb ist mein Verständnis, wie der Ansatz aus folgendem Paper umzusetzen ist:
https://ieeexplore.ieee.org/document/7428632

## Allgemeine Variablen
- $x_k \in \R^{n\times 1}$: Zustandsvektor, welcher von den Sensoren kommt
- $u_k\in \R^{m\times 1}$: Input (= Ausgabe des LQR). Heißt "Input", weil es die Eingabe für den Aktuator ist, also die Steuerungsanweisung.
- $\tau_k\in\R$: Delay im Schritt $k$ in Sekunden

Beim inversen Pendel ist 
- $n=4$, da $x_k=(x_\text{cart}, v_\text{cart}, x_\text{pole}, v_\text{pole})$
- $m=1$, da $u_k = u_\text{accel}$

## Gegebene Parameter aus der normalen LQR-Formulierung ohne Delay
- $A\in\R^{n\times n}$
- $B\in\R^{n\times m}$
- $Q\in \R^{n\times n}$
- $R\in \R^{m\times m}$

Bisher werden diese Parameter direkt in den LQR-Solver vom Matlab eingegeben, um die Matrix $K\in\R^{m\times n}$ zu berechnen.

## Parameter unter Berücksichtigung des Delays
Wir erstellen Parametermatrizen $A_z, B_z, Q_z, R_z$ als Modifikationen der gegebenen Parameter:

### $A_z$:
$$A_z = \begin{pmatrix}
\phi & \Gamma_1(\tau_k)\\
0 & 0
\end{pmatrix} \in \R^{(n+m) \times (n+m)}$$

mit
$$\phi = e^{A\cdot h} \in \R^{n\times n}$$
wobei $h\in\R$ die Sampling-Periode ist

und
$$\Gamma_1(\tau_k) = \left(\int_{h-\tau_k}^h e^{A\cdot s}~\mathrm{d}s\right)\cdot B  \in \R^{n \times m}$$
wir integrieren also über das letzte $\tau_k$-große Stück der Sampling-Periode.

*Anmerkung: Das Paper geht bei $A_z$ wohl von $m=1$ aus. Die Matrix hat nämlich immer $n+1$ Zeilen, statt den generischen $n+m$.*

### $B_z$:
$$B_z = \begin{pmatrix}
\Gamma_0(\tau_k)\\
1
\end{pmatrix} \in \R^{(n+m) \times m}$$

mit
$$\Gamma_0(\tau_k) = \left(\int_{0}^{h-\tau_k} e^{A\cdot s}~\mathrm{d}s\right)\cdot B \in \R^{n \times m}$$
wir integrieren also über das erste $\tau_k$-große Stück der Sampling-Periode.

*Anmerkung: Das Paper geht bei $B_z$ wohl von $m=1$ aus. Die Matrix hat nämlich immer $n+1$ Zeilen, statt den generischen $n+m$.*

### $Q_z$:
$$Q_z = \begin{pmatrix}
Q & 0\\
0 & R
\end{pmatrix} \in \R^{(n+m)\times (n+m)}$$

### $R_z$:
$$R_z = R$$
(vermutlich; $R_z$ wird im Paper nicht erwähnt)

## Erstellung des LQR-Reglers in Matlab
```matlab
% Define A_z, B_z, Q_z, R_z as described above
% ...

C_z = eye(n+m);
D_z = zeros(n+m, m);

sys_cont = ss(A_z, B_z, C_z, D_z);
sys_disc = c2d(sys_cont,Ts);

[K_z,S_z,P_z] = lqr(sys_disc,Q,R)
```

Der wichtige Output ist die Matrix $K_z \in \R^{m\times (n+m)}$

## Verwendung zur Laufzeit
Zur Laufzeit bekommen wir vom Sensor einen Vektor $x_k\in\R^{n\times 1}$ und wollen einen Input $u_k \in\R^{m\times 1}$ für den Aktuator berechnen.

Das machen wir wie folgt:

$$u_k = -K_z\cdot z_k$$
mit
$$z_k = \begin{pmatrix}
x_k\\
u_{k-1}
\end{pmatrix}\in\R^{(n+m)\times 1}$$
