using LinearAlgebra
using JLD
using Distributions
using StatsBase

PARENTDIR = abspath(@__DIR__,"..")
ReplicationSimDIR = abspath(PARENTDIR,"replication-sim") # call this SCRaM from now on...
QuartsSimDIR = abspath(PARENTDIR,"quarts-sim")
PARENTDIR in LOAD_PATH || push!(LOAD_PATH, PARENTDIR)
ReplicationSimDIR in LOAD_PATH || push!(LOAD_PATH, ReplicationSimDIR)
QuartsSimDIR in LOAD_PATH || push!(LOAD_PATH, QuartsSimDIR)
(@__DIR__) in LOAD_PATH || push!(LOAD_PATH, @__DIR__)

using LQRUtils
using PlantModels

using SimJulia
using Random
using RandomNumbers
using ProgressMeter
using Printf

using ProtoBuf
import PendulumMessage

use_quarts() = (try; ENV["USE_QUARTS"] == "1"; catch e; false; end)

if use_quarts()
    using QuartsSim
else
    using ReplicationSim
end

include("teensy_serial.jl")  # LibSerialPort + Base64 + Teensy port discovery

mutable struct ExperimentState
    sys::ControlSystem
    teensy::SerialPort
    mc::MessageCollector

    # logged states
    k::UInt32             # sampling period
    x::Matrix{Float32}    # log of received "states" (from Teensy)
    u::Vector{Float32}    # log of applied inputs as reported by Teensy
    tsim::Vector{UInt64}  # log of simulation time between steps

    x1_int::Float64       # integral of cart position, if needed
    t::UInt64             # start of simulation period

    # parameters
    ki::Float64           # integral gain (as multiple of K[1]), if needed
    u_max::Float32        # acceleration limit
    v_max::Float32        # speed limit
    suppress::Bool        # always send u=0?
end

include("sensor-proc-demo.jl")  # a sensor_proc for ReplicationSim and QuartsSim that calls step_experiment

Tmax = 180 # [s] experiment runtime

## initialize system model

poleLength = 0.6 # [m]

# periodMillis = 30; r = 0.005
periodMillis = 50; r = 0.02
# periodMillis = 80; r = 0.08

# periodMillis = 1000; r = 1.0

q = [ 1, 0.01, 1.5, 0.01 ]
# r = 0.03

# v = [ 1.5e-4, 1.5e-4, 2.5e-3, 2.5e-3 ]
# w = [ 1e-3, 1e-3, 1e-1, 2.5 ]
v = [ 1.5e-4, 1.5e-4, 2.5e-3, 2.5e-3 ]
w = [ 1e-3, 1e-2, 1e-1, 2 ]

Ts = 1e-3 * periodMillis # [s]
plant = InvertedPendulum(Ts, poleLength; Q=Diagonal(q), R=r, W=Diagonal(w),
    # C=Diagonal(ones(4))[1:3,:], V=Diagonal(v[1:3])
    C=Diagonal(ones(4))[[1,3],:], V=Diagonal(v[[1,3]])
) |> PlantModel

sys = ControlSystem(plant)

#                          n,       m, g,       h,        T,     δn,    p,   θd,   θc,    τ,   R, sys
# config = SimConfig(plant.n, plant.p, 3, plant.m, plant.Ts, 0.5e-3, 1e-2, 1e-3, 1e-4, 8e-3, 1.0, sys)
#  config = SimConfig(plant.n, plant.p, 3, plant.m, plant.Ts, 0.5e-3, 1e-1, 1e-2, 5e-2, 8e-3, 2.0, sys)
  config = SimConfig(plant.n, plant.p, 3, plant.m, plant.Ts, 0.5e-3, 1e-3, 1e-2, 1e-1, 8e-3, 3.0, sys)

#  config = SimConfig(plant.n, plant.p, 3, plant.m, plant.Ts, 0.5e-3, 1e-2, 1e-2, 1e-2, 8e-3, 1.0, sys) seed = 10 -> 0.04% unavailable (Quarts)
#  config = SimConfig(plant.n, plant.p, 3, plant.m, plant.Ts, 0.5e-3, 1e-2, 1e-2, 1e-1, 8e-3, 3.0, sys) seed = 10 -> Quarts 1.89% Scram 0.83% unavailable
#  config = SimConfig(plant.n, plant.p, 3, plant.m, plant.Ts, 0.5e-3, 1e-3, 1e-2, 1e-1, 8e-3, 3.0, sys) seed = 10, 120s -> Quarts 0.83% Scram 0.83% unavailable

### this was used:
# config = SimConfig(plant.n, plant.p, 3, plant.m, plant.Ts, 0.5e-3, 1e-3, 1e-2, 1e-1, 8e-3, 3.0, sys) seed = 10, 180s -> Quarts 0.78% Scram 0.78% unavailable
# currentrun = (result, estate) = runsim_experiment(config, 180, teensy, 10; noise_factor=0.01, share_OV=false, hold=false, quiet=false, suppress_u=false)
#
# Average availability: availability = sum([sum(r.available) for (r,e) in runs])/sum([length(r.available) for (r,e) in runs])
# Marginal LQR cost: J = [mean(r.cost)/r.config.T for (r,e) in runs]
# Maximum absolute angle: maxangle = [rad2deg(maximum(abs,filter(!isnan,e.x[3,:]))) for (r,e) in runs]
# Maximum cart range: maxrange = [(p=filter(!isnan,e.x[1,:]); maximum(p)-minimum(p)) for (r,e) in runs]
# Initial angle: [rad2deg(e.x[3,1]) for (r,e) in runs]
# [-0.9270422, 0.42295775, -0.027042199, -0.47704217, -0.027042199, -0.6270422, -0.32704213, 0.12295782, -0.32704213, -0.9270422, -0.6270422, -0.17704222, -0.47704217, -0.7770422, -0.17704222, -0.9270422, -0.32704213, -0.027042199, -1.0770421, -0.32704213]
# Experiment runtime: t = [simtime(r) for (r,e) in runs]
# 
# LQR [1e-2] cost CDF: 100*sort(J)
# Angle [°] CDF: sort(maxangle)
# Range [cm] CDF: 100*sort(maxrange)
# 

"Method for performing one send-receive step for interaction with the Teensy from within the event-based simulation"
# OV is only read -- plant_input, plant_output, and cost are modified in place
function step_experiment!(es::ExperimentState, OV::Vector{Float64}, plant_input::Vector{Float64}, plant_output::Vector{Float64}, cost::Vector{Float64})
    # calculate simulation time and cart position integral
    if es.k > 0
        es.tsim[es.k] = time_ns() - es.t
        es.x1_int += es.x[1,es.k] * es.sys.plant.Ts
    end

    # calculate output (adding integral term here, outside of regular control law...)
    # u = es.suppress ? 0f0 : Float32( clamp(OV[1] + es.ki * es.x1_int, -es.u_max, es.u_max) )
    u = es.suppress ? 0f0 : Float32( clamp(OV[1], -es.u_max, es.u_max) )

    # send ActuatorMessage with OV
    actuator_message = ProtoBuf.instantiate(PendulumMessage.Actuator)
    actuator_message.k = es.k + one(es.k)
    actuator_message.u = u
    send_message(actuator_message, es.teensy)
#    println("Sent ",actuator_message)

    # receive SensorMessage to get plant_output
    t0 = time_ns()
    while collect_message!(es.mc, es.teensy) == 0
        sleep(0.001)
        if (time_ns()-t0)*1e-9 > es.sys.plant.Ts
            @warn "Receive timeout"
            return
        end
    end
    sensor_message = get_message(es.mc, PendulumMessage.Sensor)
#    println("Received ",sensor_message)

    if isnan(sensor_message.u)
        @warn "Experiment aborted by Teensy"
        throw(InterruptException())
    end

    es.k = sensor_message.k  # TODO: check for skipped periods?
    # log received "state" (assuming for simplicity that this is the true state...)
    es.x[1,es.k] = sensor_message.x_cart
    es.x[2,es.k] = sensor_message.v_cart
    es.x[3,es.k] = sensor_message.x_pole + 0.01
    es.x[4,es.k] = sensor_message.v_pole
    # extract "actual" measurements
    plant_output[:] = es.sys.plant.C * ( es.x[:,es.k] + [es.ki * es.x1_int, 0f0, 0f0, 0f0] )
    es.u[es.k] = plant_input[1] = sensor_message.u  # get actually applied input
    # determine cost
    cost[es.k] = es.x[:,es.k]' * es.sys.plant.Q * es.x[:,es.k] +
                 2 * es.x[:,es.k]' * es.sys.plant.H * plant_input +
                 plant_input' * es.sys.plant.R * plant_input

    es.t = time_ns() # start timer for next simulation period
end

function send_stop_message(sp::SerialPort, k::UInt32=zero(UInt32))
    actuator_message = ProtoBuf.instantiate(PendulumMessage.Actuator)
    actuator_message.k = k
    actuator_message.u = NaN32
    send_message(actuator_message, sp)
#    println("Sent ",actuator_message)
end

## define method for running experiment

function runsim_experiment(config::SimConfig, until::Real, teensy::SerialPort, seed=nothing, failseed=seed; noise_factor::Float64=0.0, share_OV::Bool=true, hold::Bool=true, quiet::Bool=false, suppress_u::Bool=false)
#### SETUP FOR SIMULATION ####
    if seed isa Integer && seed > 0
        Random.seed!(seed)
    end
    # Initialize RNGs for noise variables independently
    state_rng = Xorshifts.Xoroshiro128Plus(rand(UInt64))
    measurement_rng = Xorshifts.Xoroshiro128Plus(rand(UInt64))

    quiet || println("+++ [INIT] ",config)

    simsteps = ceil(Int,until/config.T)

    OV = zeros(config.sys.plant.m)
    measurement_noise = cholzero(noise_factor*config.sys.plant.V) * randn(measurement_rng, config.sys.plant.p,simsteps+1)
    cost = zeros(simsteps)

    plant_input = copy(OV)
    OVreplica = share_OV ? plant_input : nothing

    plant_output = zeros(config.sys.plant.p)

    # @info "Setting up simulation and resources"
    sim = Simulation()
    actuator_queue = Store{ActuatorMessage}(sim)
    notify_actuator = Resource(sim)
    log_messages = Ref(0)

    ReplicaType = isdefined(Main, :ReplicationSim) ? NCSReplica : QuartsReplica

    replicas = Vector{ReplicaType}(sim, config.g, config, log_messages, simsteps, failseed)
    log_available = falses(length(replicas), simsteps)
    log_agreement = falses(length(replicas), simsteps)
    log_delay = fill(NaN, simsteps)

    log_state = fill(NaN32, config.sys.plant.n,simsteps)
    log_input = fill(NaN32, simsteps)
    log_tsim = zeros(UInt64, simsteps)
    es = ExperimentState(config.sys, teensy, MessageCollector(), zero(UInt32), log_state, log_input, log_tsim, 0.0, zero(UInt64), 0.05, 0f0, 0f0, suppress_u)

    # @info "Setting up processes"
    @process sensor_proc_demo(sim, replicas, notify_actuator, config, OV, plant_input, plant_output, es, measurement_noise, cost, hold)
    if isdefined(Main, :ReplicationSim)
        for r in replicas
            @process ReplicationSim.replica_receive_proc(sim, r)
            @process ReplicationSim.replica_compute_proc_shareIV(sim, r, actuator_queue, log_available, OVreplica)
        end
        @process ReplicationSim.actuator_proc(sim, actuator_queue, notify_actuator, log_agreement, log_delay, config, OV)
    else
        for r in replicas
            @process QuartsSim.replica_receive_proc(sim, r)
            @process QuartsSim.replica_compute_proc(sim, r, replicas, actuator_queue, log_available, OVreplica)
        end
        @process QuartsSim.actuator_proc(sim, actuator_queue, notify_actuator, log_agreement, log_delay, config, OV)
    end        

#### SETUP FOR TEENSY ####
    ## send InitMessage to Teensy

    periodMillis = round(UInt32,1e3*es.sys.plant.Ts)

    drain_input(teensy, 100, 1000) # read until no bytes available for 100ms, timeout at 1000ms

    init_message = ProtoBuf.instantiate(PendulumMessage.Init)
    init_message.magic = periodMillis
    send_message(init_message, teensy)
    @info "Initializing Teensy with period of $(periodMillis)ms"

    try
        ## receive ParameterMessage from Teensy

        while collect_message!(es.mc, teensy) == 0
            sleep(0.0001)
        end
        parameter_message = get_message(es.mc, PendulumMessage.Parameters)
        es.u_max = parameter_message.u_max
        es.v_max = parameter_message.v_max
        @info "Teensy ready with v_max=$(es.v_max)m/s and u_max=$(es.u_max)m/s²\nPlease hold pendulum upright to start experiment."
        parameter_message.magic != periodMillis && @warn "Teensy reports different sampling period of $(parameter_message.magic)ms!"

        ## run experiment (+ simulation)

        # receive initial SensorMessage (k=0)
        while collect_message!(es.mc, teensy) == 0
            sleep(0.001)
        end
        sensor_message = get_message(es.mc, PendulumMessage.Sensor)
        # initial state estimate and input from measurements
        if sensor_message.k != 0
            send_stop_message(teensy)
            println("[MESSAGE ERROR] initial sensor message with k=",sensor_message.k,"≠0")
            return
        end
        x0 = [sensor_message.x_cart, sensor_message.v_cart, sensor_message.x_pole, sensor_message.v_pole]
        plant_output[:] = es.sys.plant.C * x0
        plant_input[1] = sensor_message.u
    catch ex
        if ex isa InterruptException
            println("[INTERRUPTED]")
        else
            bt = catch_backtrace()
            @error "Exception in initialization" exception=(ex,bt)
        end
        return
    end

#### START EXPERIMENT ####
    quiet || println(" << [START]")

    runtime = NaN
    status = ""
    t0 = time_ns()
    try
        run(sim, until)
        status = "[DONE]"
    catch ex
        if ex isa InterruptException
            status = "[INTERRUPTED]"
        else
            bt = catch_backtrace()
            @error "Exception in run()" exception=(ex,bt)
            status = "[EXCEPTION]"
        end
    finally
        # Send stop message to Teensy
        send_stop_message(teensy)

        runtime = (time_ns()-t0)/1e9
        until = now(sim)
        simsteps = floor(Int,until/config.T)

        quiet || println(" >> ",status," ",
            ProgressMeter.durationstring(round(Int,runtime)),
            " for ",ProgressMeter.durationstring(round(Int,until))," simulated time (×",round(Int,100*until/runtime),"%)")

        return (
            SimResult(config, log_agreement[:,1:simsteps], log_delay[1:simsteps], cost[1:simsteps], log_available[:,1:simsteps], log_messages[], runtime),
            es
        )
    end
end

warmup_experiment(config, sp) = runsim_experiment(config, Twarmup(config), sp, 1; hold=false, quiet=false, suppress_u=true)

## run actual experiment

function open_teensy()
    global teensy = open_teensy_port()
end

function close_teensy()
    if isdefined(Main, :teensy) && teensy isa SerialPort
        close(teensy)
    end
end

atexit(close_teensy)

# open_teensy() # teensy = open_teensy_port()
# warmup_experiment(config, teensy)
#
# runs = Vector{Tuple{SimResult,ExperimentState}}()
# runstart = 11
# runidx = runstart - 1
#
# runidx += 1
# (result, estate) = runsim_experiment(config, 180, teensy, 10; noise_factor=0.01, share_OV=false, hold=false, quiet=false, suppress_u=false)
# save("run$(runidx)-180-0077.jld", "result", result, "estate", estate)
# push!(runs,(result,estate))
#
# save("runs-180_$(runstart)-$(runidx).jld","runs",runs)
#
# close_teensy() # close(teensy)

runs = Vector{Tuple{SimResult,ExperimentState}}()
runstart = 11
runidx = runstart - 1

function runforit()
    global runidx += 1
    (result, estate) = runsim_experiment(config, 180, teensy, 10; noise_factor=0.01, share_OV=false, hold=false, quiet=false, suppress_u=false)
    save("run$(runidx)-180-0077.jld", "result", result, "estate", estate)
    push!(runs,(result,estate))
end
