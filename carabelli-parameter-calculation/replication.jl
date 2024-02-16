# julia 1.1.0
(@__DIR__) in LOAD_PATH || push!(LOAD_PATH, @__DIR__)

using LinearAlgebra
using LQRUtils

"""
	drop_error(γ, V)

Calculate the modified measurement error covariance matrix for missing
components, where `V` is the original covariance matrix and `γ` is an
indicator vector with 1 (or true) entries for components that were
delivered and 0 (or false) entries for components that were dropped.
"""
function drop_error(γ, V)
	V + Diagonal(1 ./ γ)
end

"""
	update_covariance(P, A, C, W, V)

Update the Kalman filter's state estimate error covariance matrix based
on the previous covariance `P`, system matrices `A` and `C`, state noise
covariance `W` and (modified) measurement noise covariance `V` (cf.
`drop_error`).

Returns the updated covariance `P⁺` and associated Kalman gain `L`.
"""
function update_covariance(P, A, C, W, V)
	P¯ = A*P*A'+W
	L = P¯*C'/(C*P¯*C'+V)
	P⁺ = (I - L*C)*P¯
	(P⁺, L)
end

function update_covariance(P, A, C, W, V, γ)
	Vdrop = drop_error(γ, V)
	update_covariance(P, A, C, W, Vdrop)
end

function update_covariance_hist(P, A, C, W, V, γiter)
	P⁺ = copy(P)
	for γ in γiter
		P⁺ = first(update_covariance(P⁺, A, C, W, V, γ))
	end
	P⁺
end

update_covariance_hist(P, A, C, W, V, mγ::Matrix) = update_covariance_hist(P, A, C, W, V, (mγ[:,i] for i in 1:size(mγ,2)))

"""
	update_state(x̂, u, y, A, B, C, L)

Update the Kalman filter's state estimate based on the previous state
estimate `x̂` and input `u`, the measurement vector `y` (where the value
of dropped components is irrelevant and can be set to 0), and system
matrices `A`, `B`, and `C`, using the Kalman gain `L` (cf.
`update_covariance`).
"""
function update_state(x̂, u, y, A, B, C, L)
	x̂¯ = A*x̂ + B*u
	x̂⁺ = x̂¯ + L*(y - C*x̂¯)
end

"""
	expected_cost_penalty(P, K, B, R)

Calculate the expected LQR cost penalty (w.r.t. the optimum) incurred by
using a control input `u = K·x` based on a state estimate `x` with error
covariance `P`, where `K` is the LQR gain `(R + Bᵀ·Plq·B)¯¹·Bᵀ·Plq·A`
and `Plq` solves the DARE `P = AᵀPA - AᵀPB(R+BᵀPB)¯¹BᵀPA + Q`, i.e.,
`(K,Plq) = dlqrp(A,B,Q,R)`.
"""
expected_cost_penalty(P, K, Plq, B, R) = tr(K'*(R + B'*Plq*B)*K*P)

expected_cost_penalty_next(P, A, B, C, R, W, V, K, Plq, γ) = expected_cost_penalty(first(update_covariance(P, A, C, W, V, γ)), K, Plq, B, R)

expected_cost_penalty_hist(P, A, B, C, R, W, V, K, Plq, γiter) = expected_cost_penalty(update_covariance_hist(P, A, C, W, V, γiter), K, Plq, B, R)

@debug "Loaded replication.jl"
