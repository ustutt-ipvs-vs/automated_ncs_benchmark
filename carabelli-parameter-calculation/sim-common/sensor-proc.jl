### SENSOR PROCESS (one process for all sensors) ###
@resumable function sensor_proc(env::Environment, replicas::Vector{R}, notify_actuator::Resource, config::SimConfig,
                                OV::Vector{Float64}, plant_input::Vector{Float64}, plant_state::Vector{Float64},
                                state_noise::Matrix{Float64}, measurement_noise::Matrix{Float64},
                                cost::Vector{Float64}, hold::Bool=true) where R <: Replica
    k = 0
    J = 0.0
    # plant_input = zeros(config.sys.plant.m)
    while true
        sampling_timeout = timeout(env, config.T, priority=-1)
        # state_timeout = notify_actuator.level == 0 ? request(notify_actuator) : nothing # notify actuator that the current period has timed out
        # isprocessed(state_timeout) || @yield state_timeout
        # @yield request(notify_actuator)
        ## Take measurement
        measurement = config.sys.plant.C * plant_state + measurement_noise[:,k+1]
isdebuglogged() && (@printtime env; println(stderr," SENSOR k=",k," y(tₖ)=",measurement))
        for i in 1:config.m
            multicast(env, replicas, SensorMessage(k, i, measurement[i]))
        end
        notify_actuator.level == 0 && request(notify_actuator)
        # if SYNCHRONIZED
        for r in replicas
            r.notify_sync_period.level == 0 && request(r.notify_sync_period) # synchronize replicas by notifying of new sampling period
        end
        # end
        @yield sampling_timeout
        k += 1
        ## Simulate system from previous sampling time up to current sampling time and log cost
        cost[k] = plant_state' * config.sys.plant.Q * plant_state +
                  2 * plant_state' * config.sys.plant.H * plant_input +
                  plant_input' * config.sys.plant.R * plant_input
        J += cost[k]
        plant_state .= config.sys.plant.A * plant_state + config.sys.plant.B * plant_input + state_noise[:,k]
isdebuglogged() && (@printtime env; println(stderr," PLANT k=",k," J += ",cost[k]," -> ",J," x(tₖ)=",plant_state))
# println(stderr,"             applied input u(tₖ₋₁)=",plant_input)
        ## Remember received OV (written by actuator_proc) for next period
        copyto!(plant_input, OV) # plant_input may be shared with replicas...
        hold || fill!(OV, 0.0) # hold or zero input?
    end
end
