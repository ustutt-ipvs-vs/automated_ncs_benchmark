# syntax: proto2
using ProtoBuf
import ProtoBuf.meta

mutable struct Init <: ProtoType
    magic::UInt32
    Init(; kwargs...) = (o=new(); fillunset(o); isempty(kwargs) || ProtoBuf._protobuild(o, kwargs); o)
end #mutable struct Init
const __req_Init = Symbol[:magic]
const __wtype_Init = Dict(:magic => :fixed32)
meta(t::Type{Init}) = meta(t, __req_Init, ProtoBuf.DEF_FNUM, ProtoBuf.DEF_VAL, true, ProtoBuf.DEF_PACK, __wtype_Init, ProtoBuf.DEF_ONEOFS, ProtoBuf.DEF_ONEOF_NAMES, ProtoBuf.DEF_FIELD_TYPES)

mutable struct Parameters <: ProtoType
    magic::UInt32
    u_max::Float32
    v_max::Float32
    Parameters(; kwargs...) = (o=new(); fillunset(o); isempty(kwargs) || ProtoBuf._protobuild(o, kwargs); o)
end #mutable struct Parameters
const __req_Parameters = Symbol[:magic,:u_max,:v_max]
const __wtype_Parameters = Dict(:magic => :fixed32)
meta(t::Type{Parameters}) = meta(t, __req_Parameters, ProtoBuf.DEF_FNUM, ProtoBuf.DEF_VAL, true, ProtoBuf.DEF_PACK, __wtype_Parameters, ProtoBuf.DEF_ONEOFS, ProtoBuf.DEF_ONEOF_NAMES, ProtoBuf.DEF_FIELD_TYPES)

mutable struct Sensor <: ProtoType
    k::UInt32
    x_cart::Float32
    x_pole::Float32
    v_cart::Float32
    v_pole::Float32
    u::Float32
    Sensor(; kwargs...) = (o=new(); fillunset(o); isempty(kwargs) || ProtoBuf._protobuild(o, kwargs); o)
end #mutable struct Sensor
const __req_Sensor = Symbol[:k,:x_cart,:x_pole,:v_cart,:v_pole,:u]
const __wtype_Sensor = Dict(:k => :fixed32)
meta(t::Type{Sensor}) = meta(t, __req_Sensor, ProtoBuf.DEF_FNUM, ProtoBuf.DEF_VAL, true, ProtoBuf.DEF_PACK, __wtype_Sensor, ProtoBuf.DEF_ONEOFS, ProtoBuf.DEF_ONEOF_NAMES, ProtoBuf.DEF_FIELD_TYPES)

mutable struct Actuator <: ProtoType
    k::UInt32
    u::Float32
    Actuator(; kwargs...) = (o=new(); fillunset(o); isempty(kwargs) || ProtoBuf._protobuild(o, kwargs); o)
end #mutable struct Actuator
const __req_Actuator = Symbol[:k,:u]
const __wtype_Actuator = Dict(:k => :fixed32)
meta(t::Type{Actuator}) = meta(t, __req_Actuator, ProtoBuf.DEF_FNUM, ProtoBuf.DEF_VAL, true, ProtoBuf.DEF_PACK, __wtype_Actuator, ProtoBuf.DEF_ONEOFS, ProtoBuf.DEF_ONEOF_NAMES, ProtoBuf.DEF_FIELD_TYPES)

export Init, Parameters, Sensor, Actuator
