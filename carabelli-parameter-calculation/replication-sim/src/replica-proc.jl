### NCS REPLICA PROCESSES ###

# Process for receiving and aggregating measurements
@resumable function replica_receive_proc(env::Environment, r::NCSReplica)
    @assert env === r.env
    while true
        msg = @yield get(r.sensor_queue)
        if msg.k > r.state.k
            # this should not be able to happen (recovery after crash?)
            @error "NCSReplica $(r.id) is in k=$(r.state.k) but received message from k'=$(msg.k)"
        end
        if msg.k == r.state.k
            r.RIV[msg.index] = msg.value                           # aggregate measurements
            # if r.notify_full.level == 0 && !any(isnan,r.RIV)       # notify replica_compute_proc of full measurement
            #     request(r.notify_full)
            # end
        end
    end
end

# coordinator by view
coord_id(v::Int, g::Int) = mod(v,g)+1
coord_id(r::NCSReplica) = coord_id(r.state.v, length(r.group))
coord(r::NCSReplica, v::Int) = r.group[coord_id(v, length(r.group))]
coord(r::NCSReplica) = r.group[coord_id(r)]

iscoord(r::NCSReplica) = r.id == r.state.cid

# ReceiveIV function
function receive!(IV::Vector{Float64},RIV::Vector{Float64})
    copyto!(IV,RIV)
    fill!(RIV,NaN)
end

# shrink or repeat a to match length of b
trim(a::AbstractVector,b::AbstractVector) = repeat(a,ceil(Int,length(b)/length(a)))[1:length(b)]

function iscurrent(r::NCSReplica, msg::T) where T<:Union{Propose,Decide,Acknowledge}
    r.state.v == msg.v && r.state.k == msg.kc
end

function iscurrent(r::NCSReplica, msg::Estimate)
    r.state.v == msg.v
end

iscurrent(r::NCSReplica, ::ViewchangeFailed) = false

function accept!(r::NCSReplica, msg::T) where T<:Union{Propose,Decide}
# if msg.k < r.state.k - 1
#     @printtime r.env; printreplica(r); println(stderr," k=",r.state.k," accepting ",split(string(typeof(msg)),'.')[end]," with ke=",msg.k)
# end
    r.state.vp, r.state.kp, r.state.ke = msg.v, msg.kc, msg.k
    copyto!(r.S,msg.S)
    copyto!(r.IV,msg.IV)
end

function accept!(r::NCSReplica, msg::Estimate)
# if msg.k < r.state.k - 1
#     @printtime r.env; printreplica(r); println(stderr," k=",r.state.k," accepting Estimate with ke=",msg.k)
#     println(stderr,r.estimates)
# end
    r.state.vp, r.state.kp, r.state.ke = msg.vp, msg.kp, msg.k
    copyto!(r.S,msg.S)
    copyto!(r.IV,msg.IV)
end

printreplica(r::NCSReplica) = iscoord(r) ? (r.state.viewchange ? print(stderr,"~R",r.id,"~") : print(stderr,"[R",r.id,"]")) : print(stderr," R",r.id," ")

function start_viewchange!(r::NCSReplica, v::Int, reason::AbstractString="")
    r.state.v = v
    r.state.cid = coord_id(r)
    r.state.viewchange = iscoord(r)
# @printtime r.env; printreplica(r); println(stderr," VIEWCHANGE k=",r.state.k," v=",v," ",reason)
end
start_viewchange!(r::NCSReplica, reason::AbstractString="") = start_viewchange!(r, r.state.v+1, reason)

function handle_estimate!(r::NCSReplica, m::Estimate)
    # print(stderr,"  r.state.v=",r.state.v," m.v=",m.v," available r.estimates: ",.!ismissing.(r.estimates))
    # if r.state.v == m.v #L6... (this is already done by `iscurrent`)
# if m.k < r.state.k - 1
#     @printtime r.env; printreplica(r); println(stderr," k=",r.state.k," handling estimate with ke=",m.k)
# end
        r.estimates[m.id] = m #L6...
    # @printtime r.env; printreplica(r); println(stderr," ",votes," votes collected")
        if m.kp == m.k == r.state.k # [optimization: learned of decision for current period]
    # print(stderr,"  VC opt")
            accept!(r, m)
    # @printtime r.env; println(stderr,"[R",r.id,"] VC->COORD  k=",r.state.k," v=",r.state.v," (optimization)")
            r.state.decided = true
            r.state.viewchange = false # [ALG3 L14] breaks viewchange loop
            fill!(r.estimates, missing)
        elseif count(!ismissing, r.estimates) >= (r.config.g+1)/2 # [ALG3 L7]
    # print(stderr,"  VC")
            est = maximum(skipmissing(r.estimates)) # [ALG3 L8]
            accept!(r, est) # [ALG3 L9]
    # @printtime r.env; println(stderr,"[R",r.id,"] VC->COORD  k=",r.state.k," v=",r.state.v)
            r.state.viewchange = false # [ALG3 L14] breaks viewchange loop
            fill!(r.estimates, missing)
        end
    # end
    # println(stderr)
end

# Process for computing and issuing setpoints
@resumable function replica_compute_proc(env::Environment, r::NCSReplica, actuator_queue::Store{ActuatorMessage}, log_available::BitMatrix, OVplant=nothing)
    @assert env === r.env
    m = nothing
    next_event = nothing
    viewchange_timeout = nothing

    r.state.cid = coord_id(r) # [ALG2 L4] get coordinator
    sync_period = release(r.notify_sync_period)
    # [ALG1 L10] await t_k
    @yield sync_period  # synchronize to start of period (measurement broadcast)
    t_k = now(env)
    receive_timeout = timeout(env, r.config.δn, priority=-1)
    @yield receive_timeout # TIME: t_k + δn
    in_period = true

    OVact = isnothing(OVplant) ? nothing : copy(OVplant) # the previous output vector used for state update
    OV = zeros(r.config.sys.plant.m)                  # the output vector generated by this replica

    while true # LOOP OVER PERIODS

        # START OF MAIN LOOP ITERATION
    # @printtime env; println(stderr,"=R",r.id,"= k=",r.state.k," ==============================")
        sync_period = release(r.notify_sync_period)

        # [ALG1 L11-15] perform local state update if necessary
        if r.state.ke == r.state.k - 1
    # @printtime env; printreplica(r); println(stderr,"update after sync with OV(",r.state.k-1,")=",isnothing(OVact) ? "[default]" : OVact)
            r.state.ke = r.state.k
isdebuglogged() && (@printtime env; printreplica(r); println(stderr," update with OV(",r.state.k-1,")=",isnothing(OVact) ? "[default]" : OVact," IV=",r.IV))
            update!(r.S,r.IV,r.config.sys,OVact)
            fill!(r.IV,NaN)
        elseif r.state.ke != r.state.k
            # crash failure not handled properly?
            error("NCSReplica $(r.id) is in k=$(r.state.k) but estimate still has ke=$(r.state.ke)")
        end

        # [ALG1 L17] advance period
        r.state.k = r.state.k + 1

        # sensor (plant) has "published" previously received OV as `plant_input === OVplant`, so we'll store it here
        isnothing(OVplant) || copyto!(OVact,OVplant)

        # If NCSreplica is crashed in the current period, the consensus procedure is skipped
        if fault(r) # check if NCSreplica is faulty according to Gilbert-Elliot model with delay faults, i.e., e_good = pd
    # @printtime env; printreplica(r); println(stderr," CRASHED in period k=",r.state.k)

            # TODO: discard OVact?

            ### wait for next period
            while !isprocessed(sync_period)
                get_msg = get(r.replication_queue) # discard all received messages
                @yield sync_period | get_msg
            end
            isprocessed(get_msg) || SimJulia.cancel(r.replication_queue, get_msg)
            t_k = now(env)
            receive_timeout = timeout(env, r.config.δn, priority=-1)
            ### wait for IV
            while !isprocessed(receive_timeout)
                get_msg = get(r.replication_queue) # discard all received messages
                @yield receive_timeout | get_msg
            end
            isprocessed(get_msg) || SimJulia.cancel(r.replication_queue, get_msg)
    # print(stderr," $(r.id)↓ ")
            continue
        end
    # print(stderr," $(r.id)↑ ")
        log_available[r.id,r.state.k] = true

        # [ALG1 L16] receive IV (only if NCSreplica is available)
        receive!(r.IV,r.RIV) # copy received IV (RIV) into estimate IV (and clear RIV)

    # @printtime env; printreplica(r); println(stderr," STARTING CONSENSUS for period k=",r.state.k)
        # [ALG1 L18] start consensus
        in_period = true
        r.state.decided = false
        decide_timeout = timeout(env, 3*r.config.δn, priority=-1) # Propose + Acknoledge + Decide

        while true #in_period || !isprocessed(receive_timeout) # LOOP OVER VIEWS
            if !in_period # next period already started?
                if isprocessed(receive_timeout) # still waiting for remaining message?
                    break
                end
            end
            # [ALG2 L5-7] coordinator: propose (or reinform replicas of decision)
            if iscoord(r)
                if r.state.decided # if decided, coordinator (even if not elected) relays decision for current period
    # @printtime env; printreplica(r); println(stderr," REINFORM   k=",r.state.k," v=",r.state.v," ke=",r.state.ke) # to finalize viewchange
                    multicast(env, r.group, Decide(r), r.id)
                elseif !r.state.viewchange # proposal only possible if coordinator elected
    # @printtime env; printreplica(r); println(stderr," PROPOSE    k=",r.state.k," v=",r.state.v," ke=",r.state.ke)
                    multicast(env, r.group, Propose(r), r.id)
                    fill!(r.acks,false)
                end
                if r.state.viewchange && isprocessed(viewchange_timeout) # current viewchange timed out
                    r.state.cid = -1 # hack to behave like participant
                    r.state.viewchange = false
                    fill!(r.estimates, missing)
                end
            end

            # [ALG2 L8-25] receive proposal, ack, or decision and handle appropriately
            while true #!r.state.viewchange && ( in_period || !isprocessed(receive_timeout) )
                if r.state.viewchange
                    break
                end
                if !in_period && isprocessed(receive_timeout)
    # @printtime env; printreplica(r); println(stderr," receive_timeout")
                    break
                end
    # @printtime env; printreplica(r); println(stderr," about to yield, vc=",r.state.viewchange," ip=",in_period," rtp=",isprocessed(receive_timeout))
                get_msg = get(r.replication_queue)
                # next_event = get_msg | decide_timeout | (in_period ? sync_period : receive_timeout)
                try
                    next_event = get_msg | decide_timeout
                    if in_period
                        next_event = next_event | sync_period
                    else
                        next_event = next_event | receive_timeout
                    end
                catch e
                    @printtime env; printreplica(r); print(stderr," in_period=",in_period," "); println(stderr,e)
                    println(stderr,"  get_msg: ",get_msg," ",SimJulia.state(get_msg))
                    println(stderr,"  decide_timeout: ",decide_timeout," ",SimJulia.state(decide_timeout))
                    println(stderr,in_period ? "  " : " (","sync_period: ",sync_period," ",SimJulia.state(sync_period),in_period ? "" : ")")
                    println(stderr,in_period ? " (" : "  ","receive_timeout: ",receive_timeout," ",SimJulia.state(receive_timeout),in_period ? ")" : "")
                    rethrow(e)
                end
                @yield next_event
                if isprocessed(get_msg) # received a message
    # @printtime env; printreplica(r); print(stderr," get_msg ")
                    m = value(get_msg)
    # println(stderr,split(string(typeof(m)),'.')[end])
    # println(stderr,m)
                    if r.state.v < m.v # STILL IN CURRENT VIEW?
                        start_viewchange!(r, m.v, "informed through message") # new coordinator will not repeat loop
                    end
                    if iscurrent(r, m)
                        if m isa Propose
                            # [ALG2 L10-11] accept and acknowledge proposal
                            r.state.decided || accept!(r, m) #L10
                            send(env, r.group[m.id], Acknowledge(r, m), r.id) #L11
                        elseif in_period && !r.state.decided && m isa Acknowledge
                            # [ALG2 L13-15]
                            r.acks[m.id] = true
                            if count(r.acks) >= (r.config.g+1)/2 # got quorum?
                                multicast(env, r.group, Decide(r), r.id) #L14
                            end
                        elseif m isa Decide #L17 TODO: only perform if not yet decided?
                            # [ALG2 L18-25] decide
                            accept!(r, m) #L18
                            if r.state.ke == r.state.k - 1 #L19
    # @printtime env; printreplica(r); println(stderr,"update after decide with OV(",r.state.k-1,")=",isnothing(OVact) ? "[default]" : OVact)
                                r.state.ke = r.state.k #L20
isdebuglogged() && (@printtime env; printreplica(r); println(stderr," update with OV(",r.state.k-1,")=",isnothing(OVact) ? "[default]" : OVact," IV=",r.IV))
                                update!(r.S,r.IV,r.config.sys,OVact) #L21
                                fill!(r.IV,NaN) #L22
                            end #L23
    # @printtime env; printreplica(r); println(stderr," DECIDE     k=",r.state.k," v=",r.state.v," ke=",r.state.ke," h(S)=",string(hash(r.S),base=58))
    # @printtime env; printreplica(r); println(stderr," DECIDE k=",r.state.k," v=",r.state.v," x̂(tₖ)=",r.S.estimate," l=",r.S.loss[])
                            if in_period
                                output!(OV, r.S, r.config.sys)
                                put(actuator_queue, ActuatorMessage(r.id, r.state.k, OV)) #L24
    # @printtime env; printreplica(r); println(stderr," OUTPUT OVₖ=",OV)
                            end
                            r.state.decided = true
                        elseif m isa Estimate
    # println(stderr,"  r.state.viewchange=",r.state.viewchange)
#                            @assert iscoord(r)
                            if r.state.viewchange
                                handle_estimate!(r, m)
                            end
                        end
                    end
                else
                    SimJulia.cancel(r.replication_queue, get_msg)
                end
                if isprocessed(decide_timeout) # coordinator suspected
    # @printtime env; printreplica(r); println(stderr," decide_timeout")
                    decide_timeout = timeout(env, 4*r.config.δn, priority=-1) # Estimate + Propose + Acknoledge + Decide
                    viewchange_timeout = timeout(env, r.config.δn, priority=-1) # timeout for receiving estimates
                    if !r.state.decided
                        # [ALG3] participant
                        start_viewchange!(r, "(decide_timeout)") # [ALG3 L3] new coordinator will not repeat loop
                        # r.state.v += 1 # [ALG3 L3]
                        # r.state.cid = coord_id(r) # [ALG3 L4(a)]
                        send(r.env, r.group[r.state.cid], Estimate(r), r.id) # [ALG3 L4]
                    end
                end
                if in_period && isprocessed(sync_period) # period expired on participant
    # @printtime env; printreplica(r); println(stderr," END OF PERIOD")
                    t_k = now(env)
                    receive_timeout = timeout(env, r.config.δn, priority=-1)
                    in_period = false # breaks consensus loop
                end
            end # VIEW

            viewchange_expired = false
            while true #r.state.viewchange && ( in_period || !isprocessed(receive_timeout) ) # [ALG3 L6]
                if viewchange_expired
                    break
                end
                if !r.state.viewchange
                    break
                end
                if !in_period && isprocessed(receive_timeout)
    # @printtime env; printreplica(r); println(stderr," receive_timeout (viewchange)")
                    break
                end
    # @printtime env; printreplica(r); println(stderr," about to yield (vc), vc=",r.state.viewchange," vtp=",isprocessed(viewchange_timeout)," ip=",in_period," rtp=",isprocessed(receive_timeout))
                get_msg = get(r.replication_queue) #L6...
                # next_event = get_msg | decide_timeout | viewchange_timeout | (in_period ? sync_period : receive_timeout)
                try
                    next_event = get_msg | decide_timeout
                    next_event = next_event | viewchange_timeout
                    if in_period
                        next_event = next_event | sync_period
                    else
                        next_event = next_event | receive_timeout
                    end
                catch e
                    @printtime env; printreplica(r)
                    print(stderr," (viewchange) in_period=",in_period)
                    print(stderr," r.state.viewchange=",r.state.viewchange," ")
                    println(stderr,e)
                    println(stderr,"  get_msg: ",get_msg," ",SimJulia.state(get_msg))
                    println(stderr,"  decide_timeout: ",decide_timeout," ",SimJulia.state(decide_timeout))
                    println(stderr,"  viewchange_timeout: ",viewchange_timeout," ",SimJulia.state(viewchange_timeout))
                    println(stderr,in_period ? "  " : " (","sync_period: ",sync_period," ",SimJulia.state(sync_period),in_period ? "" : ")")
                    println(stderr,in_period ? " (" : "  ","receive_timeout: ",receive_timeout," ",SimJulia.state(receive_timeout),in_period ? ")" : "")
                    rethrow(e)
                end
                @yield next_event
                if isprocessed(get_msg) # received a message
    # @printtime env; printreplica(r); println(stderr," get_msg (viewchange)")
                    m = value(get_msg) #L6...
                    # if r.state.v < m.v # STILL IN CURRENT VIEW?
                    #     start_viewchange!(r, m.v, "informed through message (viewchange)") # new coordinator will not repeat loop
                    #     # TODO: anything else to do?
                    # elseif m isa Estimate
                    if m isa Estimate
                        handle_estimate!(r, m)
    #                 else
    # @printtime env; printreplica(r); println(stderr," VIEWCHANGE k=",r.state.k," v=",r.state.v," ",split(string(typeof(m)),'.')[end]," received")
                    end
                else
                    SimJulia.cancel(r.replication_queue, get_msg)
                end
                if in_period && isprocessed(sync_period) # period expired on coordinator
                    t_k = now(env)
                    receive_timeout = timeout(env, r.config.δn, priority=-1)
    # @printtime env; printreplica(r); println(stderr," k=",r.state.k," END OF PERIOD (viewchange)")
                    in_period = false # breaks consensus loop
                    # no need to do anything else, wait for viewchange to complete or time out, after which next period will pick up
                end
                if isprocessed(viewchange_timeout) # [ALG3 L11] viewchange failed
                    # leave viewchange loop (next viewchange will be handled at top of consensus loop)
    # @printtime env; printreplica(r); println(stderr," VIEWCHANGE TIMED OUT")
                    viewchange_expired = true
                    ######################################################################
                    ## TODO: uncomment to improve synchronization of view numbers       ##
                    ##       (increases probability of successful viewchange            ##
                    ##        at expense of additional messaging)                       ##
                    #
                    # multicast_exclude_src(env, r.group, ViewchangeFailed(r), r.id) # try to synchronize current view number
                    #
                    ######################################################################
                    fill!(r.estimates, missing)
                end
                if isprocessed(decide_timeout)
    # @printtime env; printreplica(r); println(stderr," decide_timeout in viewchange (?)")
                    decide_timeout = timeout(env, 4*r.config.δn, priority=-1)
                    viewchange_timeout = timeout(env, r.config.δn, priority=-1)
                end
            end
        end
    end # OF PERIOD
end
