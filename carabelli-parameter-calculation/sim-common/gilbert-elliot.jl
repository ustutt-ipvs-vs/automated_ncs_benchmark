# module GilbertElliot

# export GilbertElliotState,
# 	GilbertElliotProcess,
# 	GilbertElliotTransition,
# 	GilbertElliotRate,
# 	rand,
# 	process_state

using Distributions
using Random

import Base: rand

@enum GilbertElliotState::Int GOOD=1 BAD=2
struct GilbertElliotProcess <: Distribution{Univariate,Discrete}
    transition::NTuple{2,Categorical{Float64}}
    rate::NTuple{2,Bernoulli{Float64}}
    state::Ref{Int}
    rng::MersenneTwister
    GilbertElliotProcess(transition::NTuple{2,Categorical{Float64}}, rate::NTuple{2,Bernoulli{Float64}}; initial::GilbertElliotState=GOOD, seed=nothing) = new(transition, rate, Ref(Int(initial)), MersenneTwister(seed))
end
GilbertElliotTransition(p::Float64, r::Float64) = (Categorical([1-p,p]), Categorical([r,1-r]))
function GilbertElliotTransition(p_bad::Float64, MTTR::Float64, T::Float64)
	r = T/MTTR
	GilbertElliotTransition( isone(p_bad) ? 1.0 : r*p_bad/(1-p_bad) , r )
end
GilbertElliotRate(e_good::Float64, e_bad::Float64) = (Bernoulli(e_good), Bernoulli(e_bad))

function rand(p::GilbertElliotProcess)::Int
    p.state[] = rand(p.rng, p.transition[p.state[]])   # update state
    rand(p.rng, p.rate[p.state[]])                     # draw according to state
end

process_state(p::GilbertElliotProcess) = GilbertElliotState(p.state[])

# transition_matrix(transition::NTuple{2,Categorical{Float64}}) = vcat((r.p' for r in transition)...)
# p_stationary(transition::NTuple{2,Categorical{Float64}}) = (p = transition[1].p[2]; r = transition[2].p[1]; [r/(p+r), p/(p+r)])
# dwell_times(transition::NTuple{2,Categorical{Float64}}) = (p = transition[1].p[2]; r = transition[2].p[1]; [1/p, 1/r])

# p_stationary(P::Matrix) = (e = eigen(Matrix(P')); normalize(vec(e.vectors[:,e.values.≈1.0]),1))
# p_stationary(m::Union{NTuple{2,Categorical{Float64}},GilbertElliotProcess}) = p_stationary(transition_matrix(m))

# end # module