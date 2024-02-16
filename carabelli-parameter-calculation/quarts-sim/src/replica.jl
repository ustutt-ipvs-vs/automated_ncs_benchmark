### QUARTS REPLICA TYPE ###
mutable struct ReplicaLabel
    current::Int  # sampling period of latest received measurements (r in Quarts paper)
    base::Int     # base sampling period used for computing state (r⁻ in Quarts paper)
end
ReplicaLabel() = ReplicaLabel(-1, -1)

struct QuartsReplica <: Replica
    id::Int
    k::ReplicaLabel
    Z::Vector{Float64}  # currently collected measurements
    S::BitVector        # indices of valid entries in Z
    H::ControllerState
    fault_process::GilbertElliotProcess
    notify_sync_period::Resource
    notify_measurements::Resource
    # notify_full::Resource
    sensor_queue::Store{SensorMessage}
    collect_vote_queue::Store{QuartsMessage}
    config::SimConfig
    log_messages::Ref{Int}
    # log_received::Matrix{Bool}      # log of directly received measurement components
    # log_collected::Matrix{Bool}     # log of received or collected measurement components
    # log_available::Vector{Bool}     # log of available sampling periods (no crash or delay fault)
    # log_crashed::Vector{Bool}       # log of sampling periods spent in crash failure
    # log_votes::Vector{Int}          # log of available votes
    # log_votes_unique::Vector{Int}   # log of unique votes
    # log_base_period::Vector{Int}    # log of base period
    # log_agreed_digest::Vector{Union{Missing,Digest}} # log of agreed digest
    # log_agree::Vector{Bool}         # log of whether Quartsreplica can agree
    # log_votesets::Vector{Vector{Union{Missing,Digest}}} # log of actual vote sets
    "Construct a `QuartsReplica` with ID `id`"
    QuartsReplica(env::Environment, id::Int, config::SimConfig, log_messages::Ref{Int}, simsteps::Integer, seed) = new(id, ReplicaLabel(),
            fill(NaN, config.m), BitArray(false for i in 1:config.m), ControllerState(config.n),
            GilbertElliotProcess(config.fault_transition, config.fault_rate, seed=seed),
            Resource(env), Resource(env),# Resource(env),
            Store{SensorMessage}(env), Store{QuartsMessage}(env),
            config,
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
function Vector{QuartsReplica}(env::Environment, number_of_replicas::Int, config::SimConfig, log_messages::Ref{Int}, simsteps::Integer, seed)
    group = Vector{QuartsReplica}(undef, number_of_replicas)
    seeds = rand(MersenneTwister(seed), UInt64, number_of_replicas)
    for i in 1:number_of_replicas
        group[i] = QuartsReplica(env,i,config,log_messages,simsteps,seeds[i])
    end
    group
end

# Fault model
drop(c::SimConfig) = Bool(rand(c.drop))
fault(r::QuartsReplica) = Bool(rand(r.fault_process))

# Default digest and message constructors
Digest(r::QuartsReplica) = Digest(r.k.base, r.S)
Query(r::QuartsReplica) = Query(r.k.current, .!(r.S))
Response(r::QuartsReplica, Q::BitVector) = (QS = Q .& r.S; Response(r.k.current, QS, r.Z[QS]))
Advertisement(r::QuartsReplica) = Advertisement(r.k.base)
Update(r::QuartsReplica) = Update(r.k.base, r.H)
Vote(r::QuartsReplica) = Vote(r.k.current, Digest(r), r.id)

deliver(dst::QuartsReplica, msg::SensorMessage) = put(dst.sensor_queue, msg)
deliver(dst::QuartsReplica, msg::QuartsMessage) = put(dst.collect_vote_queue, msg)

function count!(r::QuartsReplica, ::QuartsMessage)
    r.log_messages[] += 1
end
