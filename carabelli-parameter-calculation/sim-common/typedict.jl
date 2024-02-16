# Methods for easier serialization of custom composite types

"""
    d = typedict(x)

Convert a struct instance `x` to a `d::Dict{Symbol,Any}` with
entries for all the fields that are needed to construct a copy
of `x`, and an additional entry `d[:TYPE] == Symbol(typeof(x))`.
It defaults to `identity` to allow for recursive construction
of nested `Dict`s. Composite types that should be converted to
`Dict`s using `typedict` must implement the `cfields` method
and overload `typedict` to `typedict_struct`. Optionally,
overloading `(::Type{T})(::Dict) = typedict_recover(T,d)` for
type `T` adds a constructor performing the "inverse".
`typedict` automatically broadcasts on `AbstractArray`s.
"""
typedict(x) = x
typedict(x::AbstractArray) = typedict.(x)

typedict_struct(x::T) where T = Dict(:TYPE=>Symbol(T), (n=>typedict(getfield(x,n)) for n in cfields(T))...)

function typedict_getfield(::Type{T}, d::Dict, n::Symbol) where T
    f = d[n]
    F = fieldtype(T,n)
    if f isa F
        return f
    elseif f isa Dict
        return F(f)
    else
        throw(TypeError(Symbol(T), "typedict_getfield", Union{F,Dict}, typeof(f)))
    end
end

function typedict_recover(::Type{T}, d::Dict)::T where T
    @assert d[:TYPE] == Symbol(T)
    T( (typedict_getfield(T,d,n) for n in cfields(T))... )
end

function typedict_recover(d::Dict)
    typesym::Symbol = d[:TYPE]
    T = eval(:($typesym))
    T( (typedict_getfield(T,d,n) for n in cfields(T))... )
end    
