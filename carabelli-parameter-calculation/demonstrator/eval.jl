using LinearAlgebra

PARENTDIR = abspath(@__DIR__,"..")
PARENTDIR in LOAD_PATH || push!(LOAD_PATH, PARENTDIR)
(@__DIR__) in LOAD_PATH || push!(LOAD_PATH, @__DIR__)

using LQRUtils
using PlantModels

include("teensy_serial.jl")  # LibSerialPort + Base64 + Teensy port discovery

Tmax = 60 # [s] experiment runtime

## initialize system model

poleLength = 0.6 # [m]

# periodMillis = 50 # [ms]

# periodMillis = 30; r = 0.003; dxi = 0.08
periodMillis = 50; r = 0.01; dxi = 0.1
# periodMillis = 80; r = 0.08; dxi = 0.2

q = [ 1, 0.01, 1.5, 0.01 ]
# r = 0.03

v = [ 1.5e-4, 1.5e-4, 2.5e-3, 2.5e-3 ]
w = [ 1e-3, 1e-3, 1e-1, 2.5 ]

Ts = 1e-3 * periodMillis # [s]
plant = InvertedPendulum(Ts, poleLength; Q=Diagonal(q), R=r, W=Diagonal(w),
  # C=Diagonal(ones(4))[1:3,:], V=Diagonal(v[1:3])
  C=Diagonal(ones(4))[[1,3],:], V=Diagonal(v[[1,3]])
) |> PlantModel

sys = ControlSystem(plant)

K = -sys.K
# K = [-1.0 0.0 0.0 0.0]

ki = 0.5*K[1]
# dxi = 0.1

L = dkalman(plant.A, plant.C, plant.W, plant.V)

# function limit_u(u, v_est, Ts, u_max, v_max) # do not take v_max into account for now
#   if u < -u_max
#     u = -u_max
#   elseif u > u_max
#     u = u_max
#   end
#   v = v_est + u * Ts
#   if v < -v_max
#     ...
# end

# TODO ...

## set up simulation model

# TODO ...

## set up serial interface and base64 decoder

# mc = MessageCollector()
# teensy = open_teensy_port()

## define method for running experiment

runexperiment(plant::PlantModel, K, ki, L, Tmax, Ts, mc::MessageCollector, ::Nothing; kwargs) = @warn "Must provide serial port!"

function runexperiment(plant::PlantModel, K, ki, L, Tmax, Ts, mc::MessageCollector, teensy; suppress=false, dxi=0.1)
  ## send InitMessage to Teensy

  init_message = ProtoBuf.instantiate(InitMessage)
  init_message.magic = UInt32(periodMillis)
  send_message(init_message, teensy)
  @info "Initializing Teensy with period of $(periodMillis)ms"

  ## receive ParameterMessage from Teensy

  while collect_message!(mc, teensy) == 0
    sleep(0.0001)
  end
  parameter_message = get_message(mc, ParameterMessage)
  u_max = parameter_message.u_max
  v_max = parameter_message.v_max
  @info "Teensy ready with v_max=$(v_max)m/s and u_max=$(u_max)m/s²"
  parameter_message.magic != periodMillis && @warn "Teensy reports different sampling period of $(parameter_message.magic)ms"

  ## run experiment (+ simulation)

  kmax = round(UInt32,abs(Tmax/Ts))

  # receive initial SensorMessage (k=0)
  while collect_message!(mc, teensy) == 0
    sleep(0.001)
  end
  sensor_message = get_message(mc, SensorMessage)
  # initial state estimate and input from measurements
  k = sensor_message.k
  x = [sensor_message.x_cart, sensor_message.v_cart, sensor_message.x_pole, sensor_message.v_pole]
  y = plant.C * x
  u = sensor_message.u

  x_est = Float64.(x)  # keep estimate at double precision
  x1_int = x_est[1]    # integrate cart position

  while true
    # predict state estimate
    x_est = plant.A * x_est + plant.B * u
    x1_int += clamp(x_est[1], x1_int>0 ? -Inf : -dxi, x1_int<0 ? Inf : dxi) * Ts

    # calculate output
    u = suppress ? 0f0 : Float32(clamp(first(K * x_est) + ki*x1_int,-u_max,u_max))

    actuator_message = ProtoBuf.instantiate(ActuatorMessage)
    actuator_message.k = k + one(k)
    actuator_message.u = k < kmax ? u : NaN32  # send stop message after kmax periods
    send_message(actuator_message, teensy)

    if mod(k, 10) == 0
      @info "k=$(sensor_message.k) : xₖ=$(round.(x,digits=3)) -> x̂ₖ₊₁=$(round.(x_est,digits=3)) -> uₖ₊₁=$(actuator_message.u)  (uₖ=$(sensor_message.u))"
    end

    if k >= kmax
      @info "End of experiment"
      break
    end

    # receive SensorMessage
    while collect_message!(mc, teensy) == 0
      sleep(0.001)
    end
    sensor_message = get_message(mc, SensorMessage)

    if isnan(sensor_message.u)
      @info "Aborted by Teensy"
      break
    end

    k = sensor_message.k  # TODO: check for skipped periods?
    x = [sensor_message.x_cart, sensor_message.v_cart, sensor_message.x_pole, sensor_message.v_pole]  # assume this is the true state...
    y = plant.C * x
    u = sensor_message.u  # get actually applied input

    # correct state estimate
    x_est = x_est + L*(y - plant.C * x_est)
  end
end


## run actual experiment

# mc = MessageCollector()
# teensy = open_teensy_port()
# runexperiment(plant, K, ki, L, Tmax, Ts, mc, teensy; dxi=dxi)
# close(teensy)
