### MESSAGING ###
# deliver(dst::Replica, msg) methods for protocol messages defined in protocol-specific replica.jl
# count!(r::Replica, msg) methods for protocol messages defined in protocol-specific replica.jl

count!(r::R, msg) where R <: Replica = nothing

send_callback(to::AbstractEvent, dst::R, msg::T) where {T <: Message, R <: Replica} = deliver(dst, msg)
function send(env::Environment, dst::R, msg::T) where {T <: Message, R <: Replica}
    count!(dst, msg) # increase global counter for protocol messages
    if drop(dst.config)
        # @printtime env
        # println("Network: dropped ", msg, " for replica ", dst.id)
    else
        to = timeout(env, rand(dst.config.delay))
        @callback send_callback(to, dst, msg)
    end
end

"Send a `msg` to the `Replica dst` using `deliver` for self-messages and `send` for all others"
function send(env::Environment, dst::R, msg::T, srcId::Int) where {T <: Message, R <: Replica}
    dst.id == srcId ? deliver(dst, msg) : send(env, dst, msg)
end

"Multicast a `msg` to the set of `Replica`s in `dst_group` using `send` (use for senders outside of dst_group)"
function multicast(env::Environment, dst_group::AbstractVector{R}, msg::T) where {T <: Message, R <: Replica}
    for dst in dst_group
        send(env, dst, msg)
    end
end

"Multicast a `msg` to the set of `Replica`s in `dst_group` using `deliver` for the replica with ID `srcId` and `send` for all others (use for senders within of dst_group)"
function multicast(env::Environment, dst_group::AbstractVector{R}, msg::T, srcId::Int) where {T <: Message, R <: Replica}
    for dst in dst_group
        send(env, dst, msg, srcId)
    end
end

"Multicast a `msg` to the set of `Replica`s in `dst_group` except for the replica with ID `srcId` using `send` (use for senders within of dst_group)"
function multicast_exclude_src(env::Environment, dst_group::AbstractVector{R}, msg::T, srcId::Int) where {T <: Message, R <: Replica}
    for dst in dst_group
        dst.id == srcId || send(env, dst, msg)
    end
end
