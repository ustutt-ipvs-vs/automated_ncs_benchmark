abstract type ReplicationMessage <: Message end
struct Propose <: ReplicationMessage
    id::Int             # sender ID
    v::Int              # view number
    kc::Int             # consensus period
    k::Int              # estimate period
    S::ControllerState  # estimate state
    IV::Vector{Float64} # estimate input vector
end
struct Acknowledge <: ReplicationMessage
    id::Int             # sender ID
    v::Int              # view number
    kc::Int             # consensus period
end
struct Decide <: ReplicationMessage
    id::Int             # sender ID
    v::Int              # view number
    kc::Int             # consensus period
    k::Int              # decision period
    S::ControllerState  # decided state
    IV::Vector{Float64} # decided input vector
end
struct Estimate <: ReplicationMessage
    id::Int             # sender ID
    v::Int              # (new) view number
    vp::Int             # last proposal view
    kp::Int             # last proposal period
    k::Int              # estimate period
    S::ControllerState  # estimate state
    IV::Vector{Float64} # estimate input vector
    loss::Float64       # expected loss of this estimate w.r.t. OVₖ (k==k_consensus ? S.loss[] : predict_loss(S.covariance, IV))
end

# default estimate ordering by view number and proposal period
isless(a::Estimate,b::Estimate) = isless((a.vp,a.kp),(b.vp,b.kp))

# state-consistent optimization: order admissible estimates by expected performance
# isless(a::Estimate,b::Estimate) = isless((a.vp,a.kp,-a.loss),(b.vp,b.kp,-b.loss))

# non-state-consistent optimization: order estimates only by view number and expected performance
# isless(a::Estimate,b::Estimate) = isless((a.vp,-a.loss),(b.vp,-b.loss))

# wild optimization: order estimates only by expected performance (can no longer guarantee output consistency)
# isless(a::Estimate,b::Estimate) = isless(-a.loss,-b.loss)

# import ReplicationSim: isless, Estimate
# ReplicationSim.isless(a::Estimate,b::Estimate) = isless((a.vp,a.kp),(b.vp,b.kp))
# ReplicationSim.isless(a::Estimate,b::Estimate) = isless((a.vp,a.kp,-a.loss),(b.vp,b.kp,-b.loss))
# ReplicationSim.isless(a::Estimate,b::Estimate) = isless((a.vp,-a.loss),(b.vp,-b.loss))
# ReplicationSim.isless(a::Estimate,b::Estimate) = isless(-a.loss,-b.loss)

mergeIV(estimates::AbstractVector{Union{Missing,Estimate}}) = mergeIV( e.IV for e in skipmissing(estimates) )

# message to share local IV with other replicas
struct ShareIV <: ReplicationMessage
    id::Int             # sender ID
    k::Int              # IV period
    IV::Vector{Float64} # estimate input vector
end

# message to inform participants of failed viewchange to synchronize view number for next viewchange
struct ViewchangeFailed <: ReplicationMessage
    id::Int             # sender ID
    v::Int              # view number (of failed viewchange)
end
