### STATE TYPE ###
struct ControllerState
    estimate::Vector{Float64}
    covariance::Matrix{Float64}
    loss::Ref{Float64} # estimated quality loss
end
ControllerState(n::Integer) = ControllerState(zeros(n),Matrix{Float64}(I,n,n),Ref{Float64}(0.0))
hash(c::ControllerState, h::UInt) = hash(hash(c.estimate,hash(c.covariance)),h)
function copyto!(c1::ControllerState, c2::ControllerState)
    copyto!(c1.estimate,c2.estimate)
    copyto!(c1.covariance,c2.covariance)
    c1.loss[] = c2.loss[]
end

# indicator vector (false for NaN entries) of vector of reals
indicator(IV::AbstractVector{N}) where N<:Real = BitVector(.!isnan.(IV))

# measurement noise covariance matrix with large entries for missing (NaN) values (big M)
Vloss(ind::BitVector, M=1e12) = Diagonal(M*.!ind)
Vloss(IV, M...) = Vloss(indicator(IV), M...)

# # measurement noise covariance more consistent with Liu2004 (only relevant for non-zero off-diagonal elements in V)
# function Vloss(V, IV, M=1e12)
#     Vl = copy(V)
#     mc = findall(isnan, IV)
#     for i in mc, j in mc
#         Vl[i,j] = i==j ? M : zero(eltype(V))
#     end
#     Vl
# end

# Kalman observer gain matrix `L` for error covariance `P` and input or indicator vector `IV`
function kalman_gain(P::AbstractMatrix, IV, sys::ControlSystem)
    V = sys.plant.V + Vloss(IV) # covariance matrix for lossy measurement vector
    # V = Vloss(sys.plant.V, IV)
    return P * sys.plant.C' / (sys.plant.C * P * sys.plant.C' + V) # Kalman gain
end
kalman_gain(S::ControllerState, IV, sys) = kalman_gain(S.covariance, IV, sys)

# in-place predictive Kalman update of error covariance `P` for `ILC = I - L * sys.plant.C`
# ("predictive" meaning that innovation precedes time update)
function update_covariance!(P::AbstractMatrix, ILC, sys::ControlSystem)
    P .= ILC * P # covariance innovation
    P .= (P + P')/2 # covariance symmetrization
    P .= sys.plant.A * P * sys.plant.A' + sys.plant.W # covariance update
end
update_covariance!(S::ControllerState, ILC, sys) = update_covariance!(S.covariance, ILC, sys)
update_covariance(P::AbstractMatrix, ILC, sys) = update_covariance!(copy(P), ILC, sys)
update_covariance(S::ControllerState, ILC, sys) = update_covariance(S.covariance, ILC, sys)

# in-place predictive Kalman update of estimate `x̂`, input vector `IV`, observer gain `L`, `ILC = I - L * sys.plant.C`, and previous output vector `OV`
# ("predictive" meaning that innovation precedes time update)
function update_estimate!(x̂::AbstractVector, IV, L, ILC, sys::ControlSystem, OV::AbstractVector = -sys.K * x̂)
    x̂ .= ILC * x̂ + L * replace(IV, NaN => 0.0) # state innovation
    x̂ .= sys.plant.A * x̂ + sys.plant.B * OV # state update
end
update_estimate!(x̂::AbstractVector, IV, L, ILC, sys::ControlSystem, ::Nothing) = update_estimate!(x̂, IV, L, ILC, sys)
update_estimate!(S::ControllerState, IV, L, ILC, sys::ControlSystem, OV...) = update_estimate!(S.estimate, IV, L, ILC, sys, OV...)
update_estimate(x̂::AbstractVector, IV, L, ILC, sys::ControlSystem, OV...) = update_estimate!(copy(x̂), IV, L, ILC, sys, OV...)
update_estimate(S::ControllerState, IV, L, ILC, sys::ControlSystem, OV...) = update_estimate(S.estimate, IV, L, ILC, sys, OV...)

# expected cost penalty w.r.t. optimum due to error covariance `P`
loss(P::AbstractMatrix, sys::ControlSystem) = tr(sys.K' * (sys.plant.R + sys.plant.B' * sys.P * sys.plant.B) * sys.K * P) # note: `sys.P` is LQR cost-to-go while `P` is error covariance
loss(S::ControllerState, sys) = loss(S.covariance, sys)
function update_loss!(S::ControllerState, sys)
    S.loss[] = loss(S.covariance, sys)
end

# predicted cost penalty w.r.t. optimum after applying `IV`
predict_loss(P, IV, sys) = loss(update_covariance(P, I - kalman_gain(P, IV, sys) * sys.plant.C, sys), sys)

function update!(S::ControllerState, IV, sys::ControlSystem, OV...)
    L = kalman_gain(S, IV, sys)
    ILC = I - L * sys.plant.C
    update_covariance!(S, ILC, sys)
    update_estimate!(S, IV, L, ILC, sys, OV...)
    # update_loss!(S, sys)  ## TODO: move to replica.jl in ReplicationSim (no need to do this for Quarts)
    return S
end

update(S, IV, sys, OV...) = update!(deepcopy(S), IV, sys, OV...)

# state update for one-step predictive LQR controller - OV is (estimate of) previously applied output vector
# function update!(S::ControllerState, IV::Vector{Float64}, sys::ControlSystem, OV::Vector{Float64})
# # print(  "--- update --- ")
#     ind = BitVector(.!isnan.(IV))
# # println(ind," ")
# # println(" ", S.estimate)
# # println(" ", S.covariance)
#     # OV = - sys.K * S.estimate # assume this was the previous output vector
#     # INNOVATION STEP
#     V = sys.plant.V + Diagonal(1 ./ (ind + 1e-12*.!ind)) # covariance matrix for lossy measurement vector
#     L = S.covariance * sys.plant.C' / (sys.plant.C * S.covariance * sys.plant.C' + V) # Kalman gain
#     ILC = I - L * sys.plant.C
#     S.estimate .= ILC * S.estimate + L * replace(IV, NaN => 0.0) # state innovation
#     S.covariance .= ILC * S.covariance # covariance innovation
#     S.covariance .= (S.covariance + S.covariance')/2 # covariance symmetrization
#     # UPDATE (PREDICTION) STEP
#     S.estimate .= sys.plant.A * S.estimate + sys.plant.B * OV # state update
#     S.covariance .= sys.plant.A * S.covariance * sys.plant.A' + sys.plant.W # covariance update
# # println("--------------")
#     # QUALITY LOSS ESTIMATION
#     S.loss[] = tr(sys.K' * (sys.plant.R + sys.plant.B' * sys.P * sys.plant.B) * sys.K * S.covariance)
#     return S
# end
# update!(S, IV, sys) = update!(S, IV, sys, -sys.K*S.estimate) # assume that previous output vector was -Kx̂
# update!(S, IV, sys, ::Nothing) = update!(S, IV, sys)

# update(S, IV, sys, OV) = update!(deepcopy(S), IV, sys, OV)
# update(S, IV, sys) = update!(deepcopy(S), IV, sys)
# update(S, IV, sys, ::Nothing) = update(S, IV, sys)

function output!(OV::AbstractVector, S::ControllerState, sys::ControlSystem)
    OV .= - sys.K * S.estimate
    return OV
end
output(S::ControllerState, sys::ControlSystem) = output!(zeros(sys.plant.m),S,sys)

### MESSAGE TYPES ###
abstract type Message end

struct SensorMessage <: Message
    k::Int
    index::Int # component index == sensor ID
    value::Float64
end

struct ActuatorMessage <: Message
    id::Int # sender replica ID
    k::Int
    OV::Vector{Float64}
end

# function for merging several IVs
function mergeIV(IVs)
    foldl( (x,y) -> ifelse.(isnan.(x),y,x), IVs )
end

function augmentIV!(IV1,IV2)
    for i in LinearIndices(IV1)
        if isnan(IV1[i])
            IV1[i] = IV2[i]
        end
    end
end
