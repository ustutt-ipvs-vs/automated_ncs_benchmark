module NCSReplicationCommon

include("gilbert-elliot.jl")
include("degenerate-distribution.jl")
include("misc.jl")
include("typedict.jl")
include("simconfig.jl")
include("message-types.jl") # common message types (ControllerState, abstract Message, SensorMessage, ActuatorMessage)
include("messaging.jl")
include("sensor-proc.jl")
include("actuator-proc.jl")
include("simresult.jl")

end