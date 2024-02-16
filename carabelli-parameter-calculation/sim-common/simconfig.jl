struct SimConfig
    ### SYSTEM PARAMETERS ###
    n::Int     # state space dimension (not specified in Quarts paper)
    m::Int     # number of sensors                             paper: 10
    g::Int     # number of replicas                            paper: 2
    h::Int     # number of actuators                           paper: 1
    T::Float64 # cycle time [s]                                paper: 20e-3

    ### FAILURE MODEL PARAMETERS ###
    δn::Float64 # netork delay [s]                             paper: 0.5e-3
    p::Float64  # network loss probability                     paper: 1e-3
    θd::Float64 # probability of "delay faults"                paper: 1e-3
    θc::Float64 # probability of crash failures                paper: 1e-4
    τ::Float64  # processing deadline [s]                      paper: 8e-3
    R::Float64  # mean time to repair from crash failures [s]  paper: 1.0

    sys::ControlSystem

    drop::Bernoulli{Float64} # = Bernoulli(p)
    delay::Distribution{Univariate,Continuous} # = Uniform(nextfloat(0.0),δn)
    processing::Distribution{Univariate,Continuous} # = Exponential(-τ/log(θd))
    fault_transition::NTuple{2,Categorical{Float64}} # = GilbertElliotTransition(θc,R,T)
    fault_rate::NTuple{2,Bernoulli{Float64}} # = GilbertElliotRate(θd,1.0)
    # fault_process = GilbertElliotProcess(fault_transition, fault_rate)

    function SimConfig(n,m,g,h,T,δn,p,θd,θc,τ,R,sys)
        @assert all( (n,m,g,h,T,δn,τ,R) .> 0 )
        @assert all( 0 .<= (p,θd,θc) .<= 1 )
        @assert n == sys.plant.n # x dimension
        @assert h == sys.plant.m # u dimension == number of actuators
        @assert m == sys.plant.p # y dimension == number of sensors
        new(n,m,g,h,T,δn,p,θd,θc,τ,R,sys,
            Bernoulli(p),
            Uniform(nextfloat(zero(δn)),prevfloat(δn)), # delay
            iszero(θd) ? Degenerate(0.0) : Exponential(-τ/log(θd)), # processing
            GilbertElliotTransition(θc,R,T), GilbertElliotRate(θd,1.0)
        )
    end
end

# Implement typedict for SimConfig
cfields(::Type{SimConfig}) = (:n,:m,:g,:h,:T,:δn,:p,:θd,:θc,:τ,:R,:sys)
typedict(x::SimConfig) = typedict_struct(x)
SimConfig(d::Dict) = typedict_recover(SimConfig,d)

function SimConfig(config::SimConfig, params::NamedTuple)
    args = ( k in keys(params) ? params[k] : getfield(config,k) for k in cfields(SimConfig) )
    SimConfig(args...)
end

cf(c::SimConfig)::Tuple = (c.n,c.m,c.g,c.h,c.T,c.δn,c.p,c.θd,c.θc,c.τ,c.R)
hash(c::SimConfig, h::UInt) = hash(hash(cf(c)),h)
==(a::SimConfig,b::SimConfig) = isequal(cf(a),cf(b))

show(io::IO, config::SimConfig) = print(io, "SimConfig(m=",config.m,", g=",config.g,
    ", p=",@sprintf("%.1E",config.p),
    ", θd=",@sprintf("%.1E",config.θd),
    ", θc=",@sprintf("%.1E",config.θc),")")
