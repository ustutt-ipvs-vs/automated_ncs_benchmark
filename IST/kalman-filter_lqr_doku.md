# Kalman-Filter und LQR: Dokumentation
Die Kalman-Filter-Implementierung orientiert sich am "Rekursiven Algorithmus" aus dem deutschen Wikipedia-Artikel: 
https://de.wikipedia.org/w/index.php?title=Kalman-Filter&oldid=238540732#Rekursiver_Algorithmus

## Konstanten
### Für Kalman-Filter
Von Matlab-Skript gegeben:
- $h\in\R$: Smallest sampling step size
- $A_\text{cont} \in\R^{4\times 4}$
- $B_\text{cont} \in\R^{4\times 1}$

Abhängig von Sampling-Periode $Ts$:
- $A = I_{4\times 4} + A_\text{cont} \cdot Ts \in\R^{4\times 4}$ (in Wikipedia: $F$)
- $B = B_\text{cont}\cdot Ts \in\R^{4\times 1}$

Sonstiges:
- $C$ (In Wikipedia: $H$):
    - Wenn $z_k \in \R^{3\times1}$: 
        $$C = \begin{pmatrix}1&0&0&0\\0&1&0&0\\0&0&1&0\end{pmatrix}\in\R^{3\times 4}$$
     - Wenn $z_k \in \R^{4\times1}$: 
        $$C = \begin{pmatrix}1&0&0&0\\0&1&0&0\\0&0&1&0\\0&0&0&1\end{pmatrix}\in\R^{4\times 4}$$
- $x_{0|-1} = (0, 0, 0, 0)^T \in\R^{4\times 1}$

Experimentell zu bestimmen:
- $Q = \sum_{i=1}^{Ts/h} A_h^{i-1}Q_0{A_h^{i-1}}^\top  \in\R^{4\times 4}$ mit
    - $Q_0$: Tuning-Parameter; selbst bestimmen.
- $Q = Q_0\cdot Ts  \in\R^{4\times 4}$ (nicht ganz Richtig, alternative Vereinfachung) mit
    - $Q_0$: Tuning-Parameter; selbst bestimmen.
- $R = R_0  \in\R^{4\times 4}$ bzw. $\R^{3\times 3}$ (hier ohne Einfluss von $Ts$) mit
    - $R_0$: Tuning-Parameter; selbst bestimmen.
- $P_{0|-1} = \sigma^2 \cdot I_{4\times 4} \in \R^{4\times 4}$ mit
    - $\sigma^2\in\R$: Selbst bestimmen.

### Für (nicht mehr LQR) Regler (jetzt Regler speziell für delays angepasst)
Von Matlab-Skript in zwei Varianten gegeben:
$$K_\text{iqc} \in \R^{4\times 1}$$
und
$$K_\text{ss} \in \R^{4\times 1}$$

## Messung im Schritt $k$
Mit 3 Parametern: 
$$z_k = \begin{pmatrix}\text{cartPos}_k\\\text{cartSpeed}_k\\\text{poleAngle}_k\\\end{pmatrix} \in \R^{3\times 1}$$

Mit 4 Parametern: 
$$z_k = \begin{pmatrix}\text{cartPos}_k\\\text{cartSpeed}_k\\\text{poleAngle}_k\\\text{poleAngleSpeed}_k\\\end{pmatrix} \in \R^{4\times 1}$$

## Werte aus Schritt $k-1$
- $x_k \in \R^{4\times 1}$: Vorherige Ausgabe des Zustands aus Kalman-Filter
- $u_{k-1} \in \R$: Letzte Ausgabe aus Regler
- $P_{k-1}\in \R^{4\times 4}$ Vorherige Ausgabe der Kovarianz aus Kalman-Filter

## Update des Kalman-Filters im Schritt $k$
1. State Prediction:
    $$x_{k|k-1} = A \cdot x_{k-1} + B \cdot u_{k-1} \in \R^{4\times 1}$$
2. State Covariance Prediction:
    $$P_{k|k-1} = A \cdot P_{k-1} \cdot A^T + Q \in \R^{4\times 4}$$
3. Kalman Gain:
    $$K_k = P_{k|k-1} \cdot C^T \cdot (C \cdot P_{k|k-1} \cdot C^T + R)^{-1} \in \R^{4\times 4} \text{ bzw. } \R^{4\times 3}$$
4. State Update:
    $$x_k = x_{k|k-1} + K_k \cdot (z_k - C \cdot x_{k|k-1}) \in \R^{4\times 1}$$
5. State Covariance Update:
    $$P_k = (I - K_k \cdot C) \cdot P_{k|k-1} \in \R^{4\times 4}$$

## Update des Eingangs mit Regler im Schritt $k$
$$u_k = K_\text{iqc}^T\cdot x_k \in \R$$
