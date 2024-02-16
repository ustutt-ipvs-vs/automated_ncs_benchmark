### NCS REPLICA TYPE ###
mutable struct ReplicaState
    k::Int              # current period
    v::Int              # current view
    ke::Int             # estimate period
    kp::Int             # last proposal period
    vp::Int             # last proposal view
    viewchange::Bool    # false → normal operation mode, true → viewchange mode
    decided::Bool
    cid::Int            # coordinator ID
end
ReplicaState() = ReplicaState(0, 0, 0, 0, 0, false, false, -1)

struct NCSReplica <: Replica
    ### static properties
    env::Environment
    group::Vector{NCSReplica}
    id::Int
    ### state information
    state::ReplicaState
    S::ControllerState  # estimate state
    IV::Vector{Float64} # estimate input vector
    RIV::Vector{Float64} # received input vector
    acks::BitVector
    estimates::Vector{Union{Missing,Estimate}}
    ### messaging
    notify_sync_period::Resource
    notify_full::Resource
    ### notify_measurements::Resource
    sensor_queue::Store{SensorMessage}
    replication_queue::Store{ReplicationMessage}
    ### configuration
    config::SimConfig
    fault_process::GilbertElliotProcess
    log_messages::Ref{Int}
    # meta::ReplicaMeta
    # log_received::Matrix{Bool}      # log of directly received measurement components
    # log_collected::Matrix{Bool}     # log of received or collected measurement components
    # log_available::Vector{Bool}     # log of available sampling periods (no crash or delay fault)
    # log_crashed::Vector{Bool}       # log of sampling periods spent in crash failure
    # log_votes::Vector{Int}          # log of available votes
    # log_votes_unique::Vector{Int}   # log of unique votes
    # log_base_period::Vector{Int}    # log of base period
    # log_agreed_digest::Vector{Union{Missing,Digest}} # log of agreed digest
    # log_agree::Vector{Bool}         # log of whether NCSreplica can agree
    # log_votesets::Vector{Vector{Union{Missing,Digest}}} # log of actual vote sets
    "Construct a `NCSReplica` with ID `id`"
    NCSReplica(env::Environment, group::Vector{NCSReplica}, id::Int, config::SimConfig, log_messages::Ref{Int}, simsteps::Integer, seed) = new(
            env,group,id,
            ReplicaState(),
            ControllerState(config.n), fill(NaN,config.m), fill(NaN,config.m),
            falses(config.g), Vector{Union{Missing,Estimate}}(missing,config.g),
            Resource(env), Resource(env),# Resource(env),
            Store{SensorMessage}(env), Store{ReplicationMessage}(env),
            config, GilbertElliotProcess(config.fault_transition, config.fault_rate, seed=seed),
            log_messages
            # fill(false,config.m,simsteps), fill(false,config.m,simsteps),
            # fill(false,simsteps),
            # fill(false,simsteps),
            # fill(0,simsteps), fill(0,simsteps),
            # fill(0,simsteps), Vector{Union{Missing,Digest}}(missing,simsteps),
            # fill(false,simsteps),
            # Vector{Vector{Union{Missing,Digest}}}(undef,simsteps)
        )
end
function Vector{NCSReplica}(env::Environment, number_of_replicas::Int, config::SimConfig, log_messages::Ref{Int}, simsteps::Integer, seed)
    group = Vector{NCSReplica}(undef, number_of_replicas)
    seeds = rand(MersenneTwister(seed), UInt64, number_of_replicas)
    for i in 1:number_of_replicas
        group[i] = NCSReplica(env,group,i,config,log_messages,simsteps,seeds[i])
    end
    group
end

# Fault model
drop(c::SimConfig) = Bool(rand(c.drop))
fault(r::NCSReplica) = Bool(rand(r.fault_process))

# Default message constructors
Propose(r::NCSReplica) = Propose(r.id,r.state.v,r.state.k,r.state.ke,deepcopy(r.S),copy(r.IV))
Acknowledge(r::NCSReplica,p::Propose) = Acknowledge(r.id,p.v,p.kc)
Decide(r::NCSReplica) = Decide(r.id,r.state.v,r.state.k,r.state.ke,deepcopy(r.S),copy(r.IV))
Estimate(r::NCSReplica) = Estimate(r.id,r.state.v,r.state.vp,r.state.kp,r.state.ke,deepcopy(r.S),copy(r.IV),r.state.ke == r.state.k ? r.S.loss[] : predict_loss(r.S.covariance,r.IV,r.config.sys))
## TODO: entirely handle loss here (also calculating `S.loss[]`) since it has been removed from the `update!` method in message-types.jl

ShareIV(r::NCSReplica) = ShareIV(r.id,r.state.k-1,copy(r.IV)) # r.state.k is consensus period, i.e. current IV has k=r.state.k-1

ViewchangeFailed(r::NCSReplica) = ViewchangeFailed(r.id,r.state.v)

deliver(dst::NCSReplica, msg::SensorMessage) = put(dst.sensor_queue, msg)
deliver(dst::NCSReplica, msg::ReplicationMessage) = put(dst.replication_queue, msg)

function count!(r::NCSReplica, ::ReplicationMessage)
    r.log_messages[] += 1
end
