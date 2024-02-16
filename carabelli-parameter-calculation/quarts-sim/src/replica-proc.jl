### QUARTS REPLICA PROCESSES ###

# Process for receiving and aggregating measurements (thread 1 in Quarts paper)
# NOTE: As the behaviour of this process is not too clearly specified in the
#       paper, there is some guesswork involved. Right now, measurements are
#       added individually to the measurement vector, and a notification is
#       triggered whenever a higher label arrives (whereupon the measurements
#       are reset), and whenever the measurement vector becomes full.
#       Possibly the Quarts authors had a different implementation in mind,
#       where the measurement vector is only updated once (they refer to time-
#       alignment in phasor data concentrators, which seems to use timeouts).
#       The current implementation enables a high degree of parallelization.
@resumable function replica_receive_proc(env::Environment, r::QuartsReplica)
    while true
        msg = @yield get(r.sensor_queue)
# @printtime env; println(stderr,"R",r.id," received mesurement (k=",msg.k,")")
        if msg.k > r.k.current
            fill!(r.Z, NaN)                                      # clear measurement vector
            fill!(r.S, false)
            # SYNCHRONIZED || r.notify_full.level == 1 && release(r.notify_full)
            r.k.current = msg.k                                  # update sampling period
            r.notify_measurements.level == 0 && request(r.notify_measurements) # notify replica_compute_proc of new measurement
        end
        if msg.k == r.k.current
            r.Z[msg.index] = msg.value                           # aggregate measurements
            r.S[msg.index] = true
        end
        # if SYNCHRONIZED || r.notify_full.level == 0 && all(r.S)
        #     request(r.notify_full)                               # notify replica_compute_proc of full measurement
        # end
    end
end

# Process for computing and issuing setpoints (thread 2 in Quarts paper)
@resumable function replica_compute_proc(env::Environment, r::QuartsReplica, replicas::Vector{QuartsReplica}, actuator_queue::Store{ActuatorMessage}, log_available::BitMatrix, OVplant=nothing)
    OVact = isnothing(OVplant) ? nothing : copy(OVplant) # the previous output vector used for state update
    votes = Vector{Union{Missing,Digest}}(missing, length(replicas))
    agreed_digest = missing
    while true
        ### WAIT FOR NEW MEASUREMENTS AND/OR ROUND ###
        # if SYNCHRONIZED
            # NOTE: This implementation of the Quartsreplica process is periodically time-driven.
            #       While less faithful to the pseudocode presented in the paper, it is
            #       possible that this setup was used by the authors for their evaulation,
            #       as this would be more in line with their Gilbert-Elliot failure model.
            #       Moreover, this is the only sensible way to ensure that all replicas
            #       execute the collect_and_vote phase at the same time.
# @printtime env; println(stderr,"R",r.id," yield release(r.notify_sync_period)")
            @yield release(r.notify_sync_period) # synchronize to start of period (measurement broadcast)
            receive_timeout = timeout(env, r.config.δn, priority=-1)
# @printtime env; println(stderr,"R",r.id," yield release(r.notify_measurements) | receive_timeout")
            @yield release(r.notify_measurements) | receive_timeout # measurements from new period or timeout occurred
            SimJulia.state(receive_timeout) == SimJulia.processed && continue # no new measurements -> wait for next period
# @printtime env; println(stderr,"R",r.id," yield receive_timeout")
            @yield receive_timeout # all replicas at t_round + δn
        # else
        #     # NOTE: This implementation of the Quartsreplica process is purely driven by the
        #     #       arrival of measurements with increasing labels. This behaviour is
        #     #       faithful to the pseudocode algorithms provided in the paper.
        #     @yield release(r.notify_measurements)
        #     # NOTE: Now we wait for "sufficient" measurements to be collected, i.e., either
        #     #       the measurement vector is full, or one maximum network delay has passed.
        #     #       We assume that this is equivalent to a relative time-alignment behaviour
        #     #       of a PDC, which was given as an example in the Quarts paper.
        #     @yield release(r.notify_full) | timeout(env, r.config.δn, priority=-1) # all measurements received or timeout
        # end

        # LOG RECEIVED MEASUREMENTS
        # r.log_received[:,r.k.current+1] = r.S

        if !isnothing(OVplant)
            copyto!(OVact,OVplant)
        end

        # PROCESS AND LOG PROCESS FAILURES
        if fault(r) # check if replica is faulty according to Gilbert-Elliot model with delay faults, i.e., e_good = pd
# @printtime env; println(stderr,"R",r.id," is faulty (k=,",r.k.current,")")
        # if fault(r) || rand(processing) > τ # alternative conditional if delay faults are not part of Gilbert-Elliot model, i.e., e_good = 0
            # r.log_available[r.k.current+1] = false
            # r.log_crashed[r.k.current+1] = (process_state(r.fault_process) == BAD)
            # r.log_collected[:,r.k.current+1] = r.S

            ## Node failure -> open-loop update
            Zzero = fill(NaN,size(r.Z))
isdebuglogged() && (@printtime env; println(stderr," R",r.id,"  update with OV(",r.k.current,")=",isnothing(OVact) ? "[default]" : OVact," IV=",Zzero))
            update!(r.H,Zzero,r.config.sys,OVact) #TODO: feed actual output vector back to all replicas

            continue
        end

# @printtime env; println(stderr,"R",r.id," is available (k=,",r.k.current,")")

        log_available[r.id,r.k.current+1] = true

        # r.log_available[r.k.current+1] = true
        # r.log_crashed[r.k.current+1] = false

        ### COLLECT ###
        all(r.S) || multicast_exclude_src(env, replicas, Query(r), r.id)
        multicast_exclude_src(env, replicas, Advertisement(r), r.id)
        collect_timeout = timeout(env, 2*r.config.δn, priority=-1)
        while r.k.base < r.k.current-1 || !all(r.S) # stale state or missing measurements?
            get_msg = get(r.collect_vote_queue, e -> e isa QuartsCollectMessage)
            @yield get_msg | collect_timeout
            if SimJulia.state(get_msg) == SimJulia.processed
                msg = value(get_msg)
                if msg isa Query && msg.k == r.k.current #----------------- Query
                    multicast_exclude_src(env, replicas, Response(r, msg.Q), r.id)
                elseif msg isa Response && msg.k == r.k.current #---------- Response
                    r.S[msg.I] .= true
                    r.Z[msg.I] = msg.P
                elseif msg isa Advertisement && msg.ks < r.k.base #--------- Advertisement
                    multicast_exclude_src(env, replicas, Update(r), r.id)
                elseif msg isa Update && msg.ks > r.k.base #---------------- Update
                    copyto!(r.H.estimate, msg.H.estimate)
                    copyto!(r.H.covariance, msg.H.covariance)
                    r.k.base = msg.ks
                end
            else
                SimJulia.cancel(r.collect_vote_queue, get_msg)
            end
            if SimJulia.state(collect_timeout) == SimJulia.processed
                break
            end
        end
        # r.log_collected[:,r.k.current+1] = r.S

        ### VOTE ###
        multicast(env, replicas, Vote(r), r.id) # multicast votes to all replicas (including self)
        fill!(votes, missing)
        agreed_digest = missing
        vote_timeout = timeout(env, 3*r.config.δn, priority=-1)
        while true
            get_msg = get(r.collect_vote_queue, e -> e isa QuartsVoteMessage)
            @yield get_msg | vote_timeout
            if SimJulia.state(get_msg) == SimJulia.processed
                msg = value(get_msg)
                if msg isa Query && msg.k == r.k.current #----------------- Query
                    multicast_exclude_src(env, replicas, Response(r, msg.Q), r.id)
                elseif msg isa Advertisement && msg.ks < r.k.base #--------- Advertisement
                    multicast_exclude_src(env, replicas, Update(r), r.id)
                elseif msg isa Vote && msg.k == r.k.current && ismissing(votes[msg.srcId]) #-------------- Vote
                    votes[msg.srcId] = msg.digest
                    f_0 = count(ismissing, votes)           # number of votes still missing

                    if f_0 <= div(length(replicas),2)       # at least half of the group size are needed
                        votes_available = skipmissing(votes)    # available votes

                        votes_unique = unique(votes_available)
                        vote_counts = collect(count(v .== votes_available) for v in votes_unique)
                        i_sort = sortperm(vote_counts, rev=true)
                        votes_unique = votes_unique[i_sort]
                        vote_counts = vote_counts[i_sort]

                        f_mc = vote_counts[1]                   # frequency of most common digest(s)
                        i_smc = findfirst(c->c<f_mc,vote_counts)
                        if i_smc == nothing
                            S_mc = votes_unique
                            f_smc = 0
                        else
                            S_mc = votes_unique[1:i_smc-1]
                            f_smc = vote_counts[i_smc]          # frequency of second most common digest(s)
                        end

                        if f_0 == 0
                            agreed_digest = maximum(S_mc)
                            break
                        elseif length(S_mc) == 1
                            if f_mc > f_smc + f_0
                                agreed_digest = S_mc[1]
                                break
                            elseif f_mc == f_smc + f_0
                                if f_smc != 0
                                    i_tmc = findfirst(c->c<f_smc,vote_counts)
                                    S_smc = (i_tmc == nothing) ? votes_unique[i_smc:end] : votes_unique[i_smc:i_tmc-1]
                                    if isless(maximum(S_smc),S_mc[1])
                                        agreed_digest = S_mc[1]
                                        break
                                    end
                                else
                                    if S_mc[1].k == r.k.current - 1 && all(S_mc[1].S) # full digest
                                        agreed_digest = S_mc[1]
                                        break
                                    end
                                end
                            end
                        end
                    end

                end
            else
                SimJulia.cancel(r.collect_vote_queue, get_msg)
            end
            if SimJulia.state(vote_timeout) == SimJulia.processed
                break
            end
        end

        # LOG VOTES ETC.
        # r.log_votes[r.k.current+1] = count(.!ismissing.(votes))
        # r.log_votes_unique[r.k.current+1] = length(unique(skipmissing(votes)))
        # r.log_base_period[r.k.current+1] = r.k.base
        # r.log_agreed_digest[r.k.current+1] = agreed_digest

        ### COMPUTE ###
        if !ismissing(agreed_digest) && agreed_digest.k == r.k.base && all(r.S[agreed_digest.S])
            # r.log_agree[r.k.current+1] = true
            #TODO: check ready_to_compute (???)

            # discard measurements that are not part of the agreed digest
            r.S .&= agreed_digest.S
            r.Z[.!agreed_digest.S] .= NaN

            # update state
isdebuglogged() && (@printtime env; println(stderr," R",r.id,"  update with OV(",r.k.current,")=",isnothing(OVact) ? "[default]" : OVact," IV=",r.Z))
            update!(r.H,r.Z,r.config.sys,OVact) #TODO: feed actual output vector back to all replicas

            # compute and issue setpoints
            put(actuator_queue, ActuatorMessage(r.id, r.k.current+1, output(r.H,r.config.sys)))

            r.k.base = r.k.current
        else
            ## No agreement -> open-loop update
            Zzero = fill(NaN,size(r.Z))
isdebuglogged() && (@printtime env; println(stderr," R",r.id,"  update with OV(",r.k.current,")=",isnothing(OVact) ? "[default]" : OVact," IV=",Zzero))
            update!(r.H,Zzero,r.config.sys,OVact) #TODO: feed actual output vector back to all replicas
            # r.log_agree[r.k.current+1] = false
        end
    end
end
