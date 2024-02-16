module QuartsSim

using Distributions
using ResumableFunctions
using SimJulia
using LinearAlgebra
using Random
using RandomNumbers
using Printf
using ProgressMeter

PARENTDIR = abspath(@__DIR__,"..")
PARENTDIR in LOAD_PATH || push!(LOAD_PATH, PARENTDIR)

using LQRUtils
using PlantModels

import Base: show, hash, ==, isless, length, iterate, copyto!

# const SYNCHRONIZED = true # synchronize replicas to rounds? (makes sense for aligned collect_and_vote)

export SimConfig, SimResult, typedict,
    simsteps, simtime,
    availability, confint_bernoulli, ci_percent,
    steps_to_repair, time_to_repair, mttr,
    steps_between_failures, time_between_failures, mtbf,
    messages_per_second,
    runsim, getmem,
    Twarmup,
    @printtime, isdebuglogged, multicast,
    Replica, QuartsReplica, SensorMessage, ActuatorMessage

include("../sim-common/gilbert-elliot.jl")
include("../sim-common/degenerate-distribution.jl")

include("../sim-common/misc.jl")
include("../sim-common/typedict.jl")
include("../sim-common/simconfig.jl")
include("../sim-common/message-types.jl") # common message types (ControllerState, abstract Message, SensorMessage, ActuatorMessage)
include("src/message-types.jl") # protocol-specific message types
include("src/replica.jl")
include("../sim-common/messaging.jl")
include("../sim-common/sensor-proc.jl")
include("src/replica-proc.jl")
include("../sim-common/actuator-proc.jl")
include("../sim-common/simresult.jl")

Twarmup(config::SimConfig) = nextfloat(config.T + config.δn)

getmem() = parse(Int,String(read(`ps -p $(getpid()) -o rss=`)))
getmem(pid) = remotecall_fetch(getmem,pid)

logstr(x,fields) = string( (string(n,"=",getfield(x,n)," ") for n in fields)... )

### SIMULATION ### (failseed is the "base" seed for process failure models)
function runsim(config::SimConfig, until::Real, seed=nothing, failseed=seed; share_OV::Bool=true, hold::Bool=true, Tmin::Real=10, Tmax::Real=600, quiet::Bool=false, logfields::NTuple{N,Symbol}=(:g,:p), variant::Symbol=:default)::SimResult where N
    if seed isa Integer && seed > 0
        Random.seed!(seed)
    end
    # Initialize RNGs for noise variables independently
    state_rng = Xorshifts.Xoroshiro128Plus(rand(UInt64))
    measurement_rng = Xorshifts.Xoroshiro128Plus(rand(UInt64))

    quiet || println("+++ ",logstr(config,logfields),"[INIT] ",config)

    simsteps = ceil(Int,until/config.T)

    # @info "Setting up control system parameters"
    # @assert config.m == sys.plant.p # measurement dimension
    # @assert config.n == sys.n # this is more or less irrelevant
    # @assert config.h == sys.plant.m # input dimension

    OV = zeros(config.sys.plant.m)
    plant_state = cholzero(config.sys.plant.W) * randn(state_rng, config.sys.plant.n)
    state_noise = cholzero(config.sys.plant.W) * randn(state_rng, config.sys.plant.n,simsteps)
    measurement_noise = cholzero(config.sys.plant.V) * randn(measurement_rng, config.sys.plant.p,simsteps+1)
    cost = zeros(simsteps)

    plant_input = copy(OV)
    OVreplica = share_OV ? plant_input : nothing

    # @info "Setting up simulation and resources"
    sim = Simulation()
    actuator_queue = Store{ActuatorMessage}(sim)
    notify_actuator = Resource(sim)
    log_messages = Ref(0)
    replicas = Vector{QuartsReplica}(sim, config.g, config, log_messages, simsteps, failseed)
    log_available = falses(length(replicas), simsteps)
    log_agreement = falses(length(replicas), simsteps)
    log_delay = fill(NaN, simsteps)

    # @info "Setting up processes"
    @process sensor_proc(sim, replicas, notify_actuator, config, OV, plant_input, plant_state, state_noise, measurement_noise, cost, hold) # periodically broadcast measurements to all replicas
    for r in replicas
        @process replica_receive_proc(sim, r)
        @process replica_compute_proc(sim, r, replicas, actuator_queue, log_available, OVreplica) # make actual output vector available to replicas
    end
    @process actuator_proc(sim, actuator_queue, notify_actuator, log_agreement, log_delay, config, OV)

    # @info "Running simulation"
    # np = 20
    # @showprogress 1 "Progress " for u in (1:np)*until/np
    #     run(sim, u)
    # end

    # runtime = @elapsed run(sim, until)

    psteps = 1000
    p_prev = 0.0
    ts_prev = 0.0
    e_prev = 0.0
    sid_prev = zero(UInt)

    quiet || println(" << ",logstr(config,logfields),"[START]")

    runtime = NaN
    status = ""
    t0 = time_ns()
    try
        for s in 1:psteps
            progress = s/psteps
            run(sim, until*progress)
            elapsed = (time_ns()-t0)/1e9
            if (Tmax-Tmin)*(progress-p_prev) + (elapsed-e_prev) > Tmax # update after between Tmin and Tmax seconds depending on progress
                realtime = elapsed-e_prev
                simtime = sim.time-ts_prev
                events = sim.sid-sid_prev

                estimate = elapsed/progress

                quiet || println("  * ",logstr(config,logfields),"[",
                    @sprintf("%2.1f",100*progress),"%] ",
                    ProgressMeter.durationstring(round(Int,elapsed)),
                    " / ",ProgressMeter.durationstring(round(Int,estimate)),
                    " -> ETA ",ProgressMeter.durationstring(round(Int, estimate-elapsed)),
                    # "  MEM ",round(Int,getmem()/1024),"M",
                    "  EPS ",@sprintf("%1.2E",events/realtime),
                    "  speed ×",@sprintf("%1.2E",simtime/realtime))

                p_prev = progress
                ts_prev = sim.time
                e_prev = elapsed
                sid_prev = sim.sid
            end
        end
        status = "[DONE]"
    catch ex
        if ex isa InterruptException
            status = "[INTERRUPTED]"
        else
            bt = catch_backtrace()
            @error "Exception in runsim()" exception=(ex,bt)
            status = "[EXCEPTION]"
        end
    finally
        runtime = (time_ns()-t0)/1e9
        until = sim.time
        simsteps = floor(Int,until/config.T)

        quiet || println(" >> ",logstr(config,logfields),status," ",
            ProgressMeter.durationstring(round(Int,runtime)),
            " for ",ProgressMeter.durationstring(round(Int,until))," simulated time (×",round(Int,100*until/runtime),"%)")

        return SimResult(config, log_agreement[:,1:simsteps], log_delay[1:simsteps], cost[1:simsteps], log_available[:,1:simsteps], log_messages[], runtime)
    end
end

# function runsim!(sim::Simulation, result::SimResult, until::Real, seed=nothing)::Tuple{SimResult,Simulation}
#     simsteps = ceil(Int,until/result.config.T)
#     if simsteps <= steps(result)
#         @info "New simulation horizon does not exceed number of previous steps, returning unchanged"
#         return result, sim
#     end
#     # 
#     if seed isa Integer && seed >= 0
#         Random.seed!(seed)
#     end
# end

end # module
