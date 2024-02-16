group_agreement(log_agreement::AbstractMatrix{Bool})::BitVector = convert(BitVector,vec(any(log_agreement, dims=1)))
group_failure(log_agreement::AbstractMatrix{Bool})::BitVector = convert(BitVector,vec(all(!,log_agreement, dims=1)))

"Calculate Jeffreys confidence (i.e., credible) interval for Binomial proportion with `s` successes in `t` trials"
function confint_bernoulli(s::Integer, t::Integer; level::Real=0.95, invert::Bool=false)
    α = 1.0 - level
    if invert; s = t-s; end
    posterior = Beta(s+0.5, t-s+0.5)
    cl = s == 0 ? 0.0 : quantile(posterior,α/2)
    cu = s == t ? 1.0 : quantile(posterior,1.0-α/2)
    cl, cu
end

confint_bernoulli(v::AbstractVector{Bool}; kwargs...) = confint_bernoulli(count(v),length(v); kwargs...)
confint_bernoulli(vs::AbstractVector{T}; kwargs...) where T <: AbstractVector{Bool} = confint_bernoulli(sum(count.(vs)),sum(length.(vs)); kwargs...)

inv_confint(ci::NTuple{2,Real}) = (1-ci[2],1-ci[1])

"Calculate vector of times to repair"
function steps_to_repair(available::AbstractVector{Bool})
    k_fail = findall(available[1:end-1] .& .!available[2:end])
    k_repair = findall(.!available[1:end-1] .& available[2:end])
    if isempty(k_fail) || isempty(k_repair)
        return NaN
    end
    if k_repair[1] < k_fail[1] # start from first failure
        k_repair = k_repair[2:end]
    end
    minlen = min(length(k_repair),length(k_fail))
    k_repair[1:minlen] - k_fail[1:minlen]
end

mean_steps_to_repair(available::AbstractVector{Bool}) = mean(steps_to_repair(available))

"Calculate vector of times between failures"
function steps_between_failures(available::AbstractVector{Bool})
    k_fail = findall(available[1:end-1] .& .!available[2:end])
    k_repair = findall(.!available[1:end-1] .& available[2:end])
    if isempty(k_fail) || isempty(k_repair)
        return NaN
    end
    if k_fail[1] < k_repair[1] # start from first repair
        k_fail = k_fail[2:end]
    end
    minlen = min(length(k_repair),length(k_fail))
    k_fail[1:minlen] - k_repair[1:minlen]
end

mean_steps_between_failures(available::AbstractVector{Bool}) = mean(steps_between_failures(available))

struct SimResult
    config::SimConfig
    unavailability::Float64
    confint95::NTuple{2,Float64}
    available::BitVector
    delay::Vector{Float64}
    cost::Vector{Float64}
    replica_agreement::BitMatrix
    replica_availability::BitMatrix
    messages::Int
    runtime::Float64

    function SimResult(config::SimConfig, replica_agreement::AbstractMatrix{Bool}, delay::AbstractVector{Float64}, cost::AbstractVector{Float64}, replica_availability::AbstractMatrix{Bool}, messages::Int, runtime::Float64=NaN)
        @assert config.g == size(replica_agreement,1) == size(replica_availability,1)
        @assert length(delay) == length(cost) == size(replica_agreement,2) == size(replica_availability,2)
        available = group_agreement(replica_agreement)
        unavailability = 1.0-mean(available)
        confint95 = confint_bernoulli(available; level=0.95, invert=true)
        new(config, unavailability, confint95, available, delay, cost, convert(BitMatrix,replica_agreement), convert(BitMatrix,replica_availability), messages, runtime)
    end
end

# Implement typedict for SimResult
cfields(::Type{SimResult}) = (:config,:replica_agreement,:delay,:cost,:replica_availability,:messages,:runtime)
typedict(x::SimResult) = typedict_struct(x)
SimResult(d::Dict) = typedict_recover(SimResult,d)

show(io::IO, result::SimResult) = print(io, "SimResult(",result.config,", ",
    @sprintf("%.2E",result.unavailability)," unavail.,",
    " 95% CI ⊆ ±",@sprintf("%.1f",ci_percent(result.unavailability,result.confint95)),"%, ",
    @sprintf("%.2E",messages_per_period(result))," mpp, J=",mean(result.cost),")")

availability(result::SimResult) = mean(result.available)
confint_bernoulli(result::SimResult; kwargs...) = confint_bernoulli(result.available; kwargs...)
time_to_repair(result::SimResult) = steps_to_repair(result.available).*result.config.T
time_between_failures(result::SimResult) = steps_between_failures(result.available).*result.config.T
mttr(result::SimResult) = mean(steps_to_repair(result))*result.config.T
mtbf(result::SimResult) = mean(steps_between_failures(result))*result.config.T

availability(results::AbstractVector{SimResult}) = mean(Iterators.flatten(getfield.(results,:available)))
confint_bernoulli(results::AbstractVector{SimResult}; kwargs...) = confint_bernoulli(getfield.(results,:available); kwargs...)
mttr(results::AbstractVector{SimResult}) = mean(Iterators.flatten(time_to_repair.(results)))
mtbf(results::AbstractVector{SimResult}) = mean(Iterators.flatten(time_between_failures.(results)))

ci_percent(m::Real,ci::NTuple{2,Real}) = max(1-ci[1]/m,ci[2]/m-1)*100
simsteps(result::SimResult) = length(result.available)
simtime(result::SimResult) = simsteps(result)*result.config.T

messages_per_period(result::SimResult) = result.messages/simsteps(result)
