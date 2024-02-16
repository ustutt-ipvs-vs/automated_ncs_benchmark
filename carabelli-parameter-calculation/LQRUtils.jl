# julia 1.1.0
module LQRUtils

using LinearAlgebra

export asmat, sysdims, sysdims_full, c2d, driccati, dare, dlqrp, dlqr, dkalman, dlyap, cholzero

"Calculate a noise shaping matrix such that `z=cholzero(M)*x` has covariance `M` if x is normal distributed"
cholzero(M) = iszero(maximum(abs,M)) ? zeros(size(M)) : Matrix(cholesky(M).L)

"Reshape scalar or vector as matrix if necessary"
asmat(X::Number) = fill(X, (1,1))
asmat(X::Vector) = reshape(X, length(X), :) # treat X as column vector
asmat(X::Matrix) = X
asmat(X::AbstractVector) = asmat(Vector(X))
asmat(X::AbstractMatrix) = Matrix(X)

"Check and return state space dimensions"
function sysdims(A,B,Q,R)
    n, m = size(B)
    @assert size(A) == (n,n)
    @assert size(Q) == (n,n)
    @assert size(R) == (m,m)
    (n, m)
end
function sysdims(A,B,Q,H,R)
    n, m = sysdims(A,B,Q,R)
    @assert size(H) == (n,m)
    (n, m)
end
function sysdims_full(A,B,C,W,V,Q,R)
    n, m = sysdims(A,B,Q,R)
    p = size(C,1)
    @assert size(C,2) == n
    @assert size(W) == (n,n)
    @assert size(V) == (p,p)
    (n, m, p)
end
function sysdims_full(A,B,C,W,V,Q,H,R)
    n, m, p = sysdims_full(A,B,C,W,V,Q,R)
    @assert size(H) == (n,m)
    (n, m, p)
end


"""
    c2d(A, B, W, Q, H, R, Ts)

Discretize continuous time system matrices `A` and `B`, noise covariance matrix
`W`, and cost matrices `Q`, `H` and `R` over sampling time `Ts`. Returns tuple
of discrete-time system matrices `(Ad, Bd, Wd, Qd, Hd, Rd)`.

> Based on `c2d.m` and `lqrd.m` from the MathWorks
>
> Calculation of `Wd` based on [Blind] and [vanLoan]
"""
function c2d(A::Matrix{Float64}, B::Matrix{Float64}, W::Matrix{Float64},
        Q::Matrix{Float64}, H::Matrix{Float64}, R::Matrix{Float64}, Ts::Float64)
    n, m = sysdims(A,B,Q,H,R)
    @assert size(W) == (n,n)

    M = [    -A'     zeros(n,m)     Q          H      ;
             -B'     zeros(m,m)     H'         R      ;
          zeros(n,n) zeros(n,m)     A          B      ;
          zeros(m,n) zeros(m,m) zeros(m,n) zeros(m,m) ]
    Φ = exp(M*Ts) # Φ = [Φ₁₁ Φ₁₂; 0 Φ₂₂]
    N = n+m
    Φ₁₂ = Φ[  1:N  , N+1:2*N]
    Φ₂₂ = Φ[N+1:2*N, N+1:2*N]
    # discretize system matrices
    Ad = Φ₂₂[1:n,1:n]
    Bd = Φ₂₂[1:n,n+1:n+m]
    # discretize cost matrices
    QQ = Φ₂₂'*Φ₁₂
    QQ = (QQ+QQ')/2 # QQ = [Qd Hd; Hd' Rd]
    Qd = QQ[  1:n,   1:n]
    Hd = QQ[  1:n, n+1:N]
    Rd = QQ[n+1:N, n+1:N]
    # discretize noise covariance
    M = [-A W; zeros(n,n) A']
    Φ = exp(M*Ts) # Φ = [Φ₁₁ Φ₁₂; 0 Φ₂₂]
    Φ₁₂ = Φ[  1:n  , n+1:2*n]
    Φ₂₂ = Φ[n+1:2*n, n+1:2*n]
    Wd = Φ₂₂'*Φ₁₂
    Wd = (Wd+Wd')/2
    (Ad, Bd, Wd, Qd, Hd, Rd)
end

"""Perform one discrete-time Riccati iteration
``Pₖ₊₁ = AᵀPₖA - (AᵀPₖB + H)⋅(R + BᵀPₖB)⁻¹⋅(B'PₖA + Hᵀ) + Q``"""
driccati(P,A,B,Q,H,R) = A'*P*A - (A'*P*B+H)/(R+B'*P*B)*(B'*P*A+H') + Q
driccati(P,A,B,Q,R) = driccati(P,A,B,Q,zeros(size(B)),R)

"""
    dare(A, B, Q[, H], R)

Compute solution `P` to discrete-time algebraic Riccati equation
``P = AᵀPA - (AᵀPB + H)⋅(R + BᵀPB)⁻¹⋅(B'PA + Hᵀ) + Q`` using Schur factorization [1].

[1] A. Laub, "A Schur method for solving algebraic Riccati equations," in IEEE
Transactions on Automatic Control, vol. 24, no. 6, pp. 913-921, Dec 1979.
https://doi.org/10.1109/TAC.1979.1102178

> This method is based on `dare` from ControlSystems.jl (Copyright 2014 Jim Crist and
> other contributors, MIT license) https://github.com/JuliaControl/ControlSystems.jl
"""
function dare(A,B,Q,H,R)
    Ah = A - B/R*H'
    Qh = Q - H/R*H'
    AitQ = Ah'\Qh
    BRiBt = B*(R\B')
    Z = [Ah+BRiBt*AitQ  -BRiBt/(Ah') ;
            -AitQ         inv(Ah)'   ]
    S = schur(Z)
    S = ordschur(S, abs.(S.values).<=1.0)
    U = S.Z
    (m, n) = size(U)
    U11 = U[1:div(m,2), 1:div(n,2)]
    U21 = U[div(m,2)+1:m, 1:div(n,2)]
    P = U21/U11
    P = driccati(P,A,B,Q,H,R)
    (P+P')/2 # remove skew-symmetric artifacts
end
dare(A,B,Q,R) = dare(A,B,Q,zeros(size(B)),R)

"Calculate optimal LQR gain `K` and cost-to-go matrix `P`"
function dlqrp(A,B,Q,H,R)
    P = dare(A, B, Q, H, R)
    K = (R + B'*P*B)\(B'*P*A + H')
    (K, P)
end
dlqrp(A,B,Q,R) = dlqrp(A,B,Q,zeros(size(B)),R)

"Calculate optimal LQR gain `K`"
dlqr(A,B,Q,H,R) = first(dlqrp(A,B,Q,H,R))
dlqr(A,B,Q,R) = first(dlqrp(A,B,Q,R))

"Calculate steady-state Kalman gain `L`"
dkalman(A,C,W,V) = Matrix(dlqr(A',C',W,V)')

"""
    dlyap(A, Q)

Compute solution `X` to discrete-time Lyapunov equation ``AXAᵀ - X + Q = 0``
using vectorization.

> This method is based on `dlyap` from ControlSystems.jl (Copyright 2014 Jim Crist and
> other contributors, MIT license) https://github.com/JuliaControl/ControlSystems.jl
"""
function dlyap(A,Q)
    lhs = kron(A, conj(A))
    lhs = eye(lhs) - lhs
    X = lhs\reshape(Q, prod(size(Q)), 1)
    X = reshape(X, size(Q))
    (X+X')/2 # remove skew-symmetric artifacts
end

end # module LQRUtils
