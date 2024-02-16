using Distributions

struct Degenerate{T<:Real} <: Distribution{Univariate,Continuous}
    x::T
    Degenerate{T}(x::T) where {T} = (Distributions.@check_args(Degenerate, !isnan(x)); new{T}(x))
end
Degenerate(x::T) where {T<:Real} = Degenerate{T}(x)
Degenerate(x::Integer) = Degenerate(Float64(x))
Degenerate() = Degenerate(0.0)

Base.rand(d::Degenerate) = d.x
Base.rand(::Random.AbstractRNG, d::Degenerate) = rand(d)
Distributions.pdf(d::Degenerate{T}, x::Real) where {T<:Real} = T(Inf)*(x == d.x)
Distributions.logpdf(d::Degenerate, x::Real) = pdf(d,x)
Distributions.cdf(d::Degenerate{T}, x::Real) where {T<:Real} = T(x >= d.x)
Distributions.quantile(d::Degenerate{T}, q::Real) where {T<:Real} = iszero(q) ? T(-Inf) : d.x
Base.minimum(d::Degenerate{T}) where {T<:Real} = T(-Inf)
Base.maximum(d::Degenerate{T}) where {T<:Real} = T(Inf)
Distributions.insupport(d::Degenerate, x::Real) = !isnan(x)
Distributions.mean(d::Degenerate) = d.x
Distributions.var(d::Degenerate{T}) where {T<:Real} = zero(T)
Distributions.modes(d::Degenerate) = [d.x]
Distributions.mode(d::Degenerate) = d.x
Distributions.skewness(d::Degenerate{T}) where {T<:Real} = T(NaN)
Distributions.kurtosis(d::Degenerate{T}) where {T<:Real} = T(NaN)
Distributions.entropy(d::Degenerate{T}) where {T<:Real} = zero(T)
Distributions.mgf(d::Degenerate, t::Real) = exp(d.x * t)
Distributions.cf(d::Degenerate, t::Real) = exp(im * d.x * t)
