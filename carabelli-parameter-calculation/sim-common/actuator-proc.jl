### DUMMY ACTUATOR PROCESS (logs agreements) ###
@resumable function actuator_proc(env::Environment, actuator_queue::Store{ActuatorMessage}, notify_actuator::Resource,
                                  log_agreement::BitMatrix, log_delay::Vector{Float64}, config::SimConfig,
                                  OV::Vector{Float64})
    k = 0
    @yield release(notify_actuator) # wait for first sampling period
    k += 1
    while true # loop over sampling periods
        t_k = now(env)
        agreement_timeout = release(notify_actuator) # wait for end of sampling period
        while true # loop over received messages
            get_msg = get(actuator_queue)
            @yield agreement_timeout | get_msg
            if SimJulia.state(get_msg) == SimJulia.processed
                msg = value(get_msg)
                if msg.k == k
                    log_agreement[msg.id,k] = true
                    if isnan(log_delay[k]) # first ActuatorMessage to arrive within sampling period determines delay
                        log_delay[k] = now(env) - t_k
                        copyto!(OV, msg.OV) # store received output vector in OV (shared with sensor_proc)
isdebuglogged() && (@printtime env; println(stderr," ACTUATOR received and set OV(",k,")=",OV))
                    end
                end
            end
            if SimJulia.state(agreement_timeout) == SimJulia.processed
                SimJulia.cancel(actuator_queue, get_msg)
                # next sampling period
                k += 1
                break
            end
        end
    end
end
