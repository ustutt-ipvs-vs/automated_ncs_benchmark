### SENSOR PROCESS for experiments with demonstrator (one process for all sensors) ###
@resumable function sensor_proc_demo(env::Environment, replicas::Vector{R}, notify_actuator::Resource, config::SimConfig,
                                OV::Vector{Float64}, plant_input::Vector{Float64},
                                plant_output::Vector{Float64}, es::ExperimentState, ### <<< added for demonstrator experiments
                                measurement_noise::Matrix{Float64},
                                cost::Vector{Float64}, hold::Bool=true) where R <: Replica
    k = 0
    J = 0.0
    # plant_input = zeros(config.sys.plant.m)
    while true
        sampling_timeout = timeout(env, config.T, priority=-1)

        ## Take measurement
        # measurement = config.sys.plant.C * plant_state + measurement_noise[:,k+1]

        measurement = plant_output + measurement_noise[:,k+1] # <<< `plant_output` written from Teensy in `step_experiment!()`

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

        # # THIS DOES NOT WORK!
        # @callback SimJulia.stop_simulation(sampling_timeout) # <<< Here we suspend the simulation in order to...

        @yield sampling_timeout
        k += 1

        step_experiment!(es, OV, plant_input, plant_output, cost)
        # ... 1) send ActuatorMessage with `OV` (already written by actuator_proc???) to Teensy
        # ... 2) receive SensorMessage from Teensy and write contents to `plant_output`
        # ... 3) resume simulation until next `sampling_timeout`

isdebuglogged() && (@printtime env; println(stderr," SENSOR k=",k," OV=",OV," after resuming"))

        ## Remember received OV
        hold || fill!(OV, 0.0) # hold or zero input?
    end
end
