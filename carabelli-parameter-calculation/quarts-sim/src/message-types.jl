indices(v::Vector{Float64}) = BitArray(.!isnan.(v))

struct Digest
    k::Int        # base sampling period
    S::BitVector  # available input components
end
Digest(k::Int, v::Vector{Float64}) = Digest(k,indices(v))
hash(d::Digest, h::UInt) = hash(hash(d.k,hash(d.S)),h)
==(a::Digest,b::Digest) = isequal(a.k,b.k) && isequal(a.S,b.S)
isless(a::Digest,b::Digest) = isless(a.k,b.k) || (isequal(a.k,b.k) && isless(a.S,b.S))
length(::Digest) = 1
iterate(d::Digest) = (d,1)
iterate(::Digest,::Any) = nothing

abstract type QuartsMessage <: Message end
struct Query <: QuartsMessage
    k::Int
    Q::BitVector # requested (missing) indices
end
struct Response <: QuartsMessage
    k::Int
    I::BitVector       # provided indices
    P::Vector{Float64} # values for provided indices (length(P) == sum(I))
end
struct Advertisement <: QuartsMessage
    ks::Int
end
struct Update <: QuartsMessage
    ks::Int
    H::ControllerState
end
struct Vote <: QuartsMessage
    k::Int
    digest::Digest
    srcId::Int # sender ID
end

QuartsCollectMessage = Union{Query,Response,Advertisement,Update}
QuartsVoteMessage = Union{Vote,Query,Advertisement}
