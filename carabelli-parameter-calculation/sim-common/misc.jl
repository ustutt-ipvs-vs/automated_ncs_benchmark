using Logging

islogged(level::LogLevel) = Logging.current_logger().min_level <= level
isdebuglogged() = islogged(Logging.Debug)
isinfologged() = islogged(Logging.Info)
iswarnlogged() = islogged(Logging.Warn)
iserrorlogged() = islogged(Logging.Error)

show(io::IO,bv::BitVector) = show(io,bitstring(bv))

macro printtime(env)
    esc(:(@printf stderr "%10.6fs " now($env)))
end

function cbecho(ev::AbstractEvent)
   @printtime ev.bev.env
   print("[EVENT] processing ", ev)
   print(", typeof(value)=",typeof(ev.bev.value))
   print(", value=",ev.bev.value)
   println()
end

isprocessed(ev::AbstractEvent) = SimJulia.state(ev) == SimJulia.processed
isprocessed(::Nothing) = true  # consider uninitialized events as processed in order to assign new event

abstract type Replica end
