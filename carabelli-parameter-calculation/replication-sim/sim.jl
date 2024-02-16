# ../julia-1.1.0/bin/julia -p8 -L load-sim.jl

include("load-sim.jl")

if false
    sims = Vector{SimResult}()
    #                  n,  m, g, h,     T,     δn,    p,   θd,   θc,    τ,   R
    config = SimConfig(2, 10, 2, 1, 20e-3, 0.5e-3, 1e-3, 1e-3, 1e-4, 8e-3, 1.0)
    result = runsim(config, 100)
    push!(sims, result)
end

# params = [(2,5e-2),(3,5e-2),(4,5e-2),(5,5e-2),(5,1e-4)]

if false
    params = [
        (2,1e-4),(2,1e-3),(2,1e-2),(2,5e-2),(2,0.1),(2,0.25),(2,0.5),
        (3,1e-4),(3,1e-3),(3,1e-2),(3,5e-2),(3,0.1),(3,0.25),(3,0.5),
        (4,1e-4),(4,1e-3),(4,1e-2),(4,5e-2),(4,0.1),(4,0.25),(4,0.5),
        (5,1e-4),(5,1e-3),(5,1e-2),(5,5e-2),(5,0.1),(5,0.25),(5,0.5)
    ]
    sims = prunsim(params, 60*60)
    # sims = pmap(params; on_error=identity) do (g,p)
    #     println("<<< g=",g,", p=",p," [START]")
    #     config = SimConfig(2, 10, g, 1, 20e-3, 0.5e-3, p, 1e-3, 1e-4, 8e-3, 1.0)
    #     result = runsim(config, 60)
    #     println(">>> g=",g,", p=",p," [DONE] ",round(Int,result.runtime),"s")
    #     result
    # end
    using DataFrames
    df = DataFrame((g=r.config.g, p=r.config.p, u=r.unavailability, cl95=r.confint95[1], cu95=r.confint95[2], t=r.runtime) for r in values(sims))
    df = DataFrame((g=r.config.g, p=r.config.p, u=r.unavailability, cip=ci_percent(r.unavailability,r.confint95), t=r.runtime, sim=r) for r in values(sims))
    by(df,[:g,:p]) do d
        (u=1.0-availability(d.sim), cip=ci_percent(u,confint_bernoulli(d.sim,invert=true)))
    end
    sort!(df,(:g,:p))
    # using JLD
    # save("sims.jld","sims",sims)
    # sims = load("sims.jld","sims")
    #
    # using Serialization
    # serialize("sims.jls",sims)
    # sims = deserialize("sims.jls")
    #
    # simtds = typedict(sims)
    # sims = SimResult.(simtds)
end

if false
    config = SimConfig(2, 10, 5, 1, 20e-3, 0.5e-3, 1e-4, 1e-3, 1e-4, 8e-3, 1.0)
    runsim(config,200,Tmin=2,Tmax=2)
end

# prunsim(params, until) = pmap(params; on_error=identity) do (g,p)
#     config = SimConfig(2, 10, g, 1, 20e-3, 0.5e-3, p, 1e-3, 1e-4, 8e-3, 1.0)
#     result = runsim(config, until; Tmin=10, Tmax=60, quiet=false)
#     result
# end

if false
    using DataFrames
    baseconfig = SimConfig(2, 10, 3, 1, 20e-3, 0.5e-3, 0.0, 0.0, 0.0, 8e-3, 1.0)
    params = [(p=0.01,),(p=0.05,),(p=0.1,),(p=0.2,),(p=0.3,),(p=0.4,),(p=0.5,)]
    # params = [(p=0.6,),(p=0.7,),(p=0.8,),(p=0.9,)]
    prunsim(baseconfig, fill(NamedTuple(),nworkers()), 0.1) # warmup
    sims = prunsim(baseconfig, params, 30*60)
end

# prunsim(baseconfig, params, until) = pmap(params; on_error=identity) do p
#     config = SimConfig(baseconfig, p)
#     result = runsim(config, until; Tmin=10, Tmax=60, quiet=false)
#     result
# end
prunsim(baseconfig, params, until, seed) = pmap(params; on_error=identity) do p
    config = SimConfig(baseconfig, p)
    result = runsim(config, until, seed; Tmin=10, Tmax=60, quiet=false)
    result
end

# config = SimConfig(baseconfig, (p=0.1,g=3,θd=1e-3,θc=1e-4))

if false
    using SimJulia
    using ResumableFunctions
    import ReplicationSim: ActuatorMessage, Replica, sensor_proc, replica_receive_proc, replica_compute_proc, actuator_proc
end
