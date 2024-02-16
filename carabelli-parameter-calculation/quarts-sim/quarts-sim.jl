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
    params = [ # (g,p)
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

prunsim(params, until) = pmap(params; on_error=identity) do (g,p)
    # config = SimConfig(2, 10, g, 1, 20e-3, 0.5e-3, p, 1e-3, 1e-4, 8e-3, 1.0)
    config = SimConfig(2, 10, g, 1, 20e-3, 0.5e-3, p, 1e-3, 1e-4, 8e-3, 1.0)
    result = runsim(config, until; Tmin=10, Tmax=60, quiet=false)
    result
end

if false
    using DataFrames
    baseconfig = SimConfig(2, 10, 3, 1, 20e-3, 0.5e-3, 0.0, 0.0, 0.0, 8e-3, 1.0)
    params = [(p=0.01,),(p=0.05,),(p=0.1,),(p=0.2,),(p=0.3,),(p=0.4,),(p=0.5,)]
    # params = [(p=0.6,),(p=0.7,),(p=0.8,),(p=0.9,)]
    prunsim(baseconfig, fill(NamedTuple(),nworkers()), 0.1) # warmup
    sims = prunsim(baseconfig, params, 30*60)
end

prunsim(baseconfig, params, until) = pmap(params; on_error=identity) do p
    config = SimConfig(baseconfig, p)
    result = runsim(config, until; Tmin=10, Tmax=60, quiet=false)
    result
end

if false
    using SimJulia
    using ResumableFunctions
    import QuartsSim: ActuatorMessage, Replica, sensor_proc, replica_receive_proc, replica_compute_proc, actuator_proc
end

# │ Row │ g     │ p       │ unavailability │ cl95        │
# │     │ Int64 │ Float64 │ Float64        │ Float64     │
# ├─────┼───────┼─────────┼────────────────┼─────────────┤
# │ 1   │ 1     │ 0.001   │ 0.001          │ 0.000514257 │
# │ 2   │ 2     │ 0.0001  │ 0.0            │ 0.0         │
# │ 3   │ 2     │ 0.001   │ 0.0003         │ 8.45005e-5  │
# │ 4   │ 2     │ 0.01    │ 0.0005         │ 0.000190812 │
# │ 5   │ 2     │ 0.1     │ 0.0061         │ 0.00471291  │
# │ 6   │ 3     │ 0.001   │ 0.0            │ 0.0         │
# │ 7   │ 4     │ 0.001   │ 0.0            │ 0.0         │

# │ Row │ g     │ p       │ u           │ cl95       │ t       │
# │     │ Int64 │ Float64 │ Float64     │ Float64    │ Float64 │
# ├─────┼───────┼─────────┼─────────────┼────────────┼─────────┤
# │ 1   │ 2     │ 0.01    │ 0.000333333 │ 3.59682e-5 │ 5.41937 │
# │ 2   │ 2     │ 0.1     │ 0.00666667  │ 0.00420726 │ 6.20988 │
# │ 3   │ 2     │ 0.2     │ 0.066       │ 0.0575316  │ 6.93474 │
# │ 4   │ 2     │ 0.4     │ 0.375667    │ 0.358463   │ 6.99231 │
# │ 5   │ 3     │ 0.01    │ 0.0         │ 0.0        │ 6.24974 │
# │ 6   │ 3     │ 0.1     │ 0.000666667 │ 0.00013856 │ 11.8644 │
# │ 7   │ 3     │ 0.2     │ 0.039       │ 0.0325111  │ 12.3663 │
# │ 8   │ 3     │ 0.4     │ 0.691333    │ 0.674625   │ 10.9775 │
# │ 9   │ 4     │ 0.01    │ 0.0         │ 0.0        │ 9.69275 │
# │ 10  │ 4     │ 0.1     │ 0.0         │ 0.0        │ 18.1341 │
# │ 11  │ 4     │ 0.2     │ 0.000333333 │ 3.59682e-5 │ 19.4374 │
# │ 12  │ 4     │ 0.4     │ 0.631       │ 0.613613   │ 15.0213 │
# │ 13  │ 5     │ 0.01    │ 0.0         │ 0.0        │ 14.0429 │
# │ 14  │ 5     │ 0.1     │ 0.0         │ 0.0        │ 26.1196 │
# │ 15  │ 5     │ 0.2     │ 0.000333333 │ 3.59682e-5 │ 28.3122 │
# │ 16  │ 5     │ 0.4     │ 0.515667    │ 0.497776   │ 23.1186 │

# runtime[s] per simtime[s]
# T = [5.41937   6.20988   6.93474   6.99231;
#      6.24974  11.8644   12.3663   10.9775;
#      9.69275  18.1341   19.4374   15.0213;
#     14.0429   26.1196   28.3122   23.1186]./60

# G = [2  2  2  2;
#      3  3  3  3;
#      4  4  4  4;
#      5  5  5  5]

# P = [0.01  0.1  0.2  0.4;
#      0.01  0.1  0.2  0.4;
#      0.01  0.1  0.2  0.4;
#      0.01  0.1  0.2  0.4]

# SimResult(SimConfig(m=10, g=2, p=1.0E-02, θd=1.0E-03, θc=1.0E-04), 3.33E-04 unavailable, 95% CI ⊆ ±367.1%)
# SimResult(SimConfig(m=10, g=2, p=1.0E-01, θd=1.0E-03, θc=1.0E-04), 6.67E-03 unavailable, 95% CI ⊆ ±51.1%)
# SimResult(SimConfig(m=10, g=2, p=2.0E-01, θd=1.0E-03, θc=1.0E-04), 6.60E-02 unavailable, 95% CI ⊆ ±14.1%)
# SimResult(SimConfig(m=10, g=2, p=4.0E-01, θd=1.0E-03, θc=1.0E-04), 3.76E-01 unavailable, 95% CI ⊆ ±4.6%)
# SimResult(SimConfig(m=10, g=3, p=1.0E-02, θd=1.0E-03, θc=1.0E-04), 0.00E+00 unavailable, 95% CI ⊆ ±NaN%)
# SimResult(SimConfig(m=10, g=3, p=1.0E-01, θd=1.0E-03, θc=1.0E-04), 6.67E-04 unavailable, 95% CI ⊆ ±220.5%)
# SimResult(SimConfig(m=10, g=3, p=2.0E-01, θd=1.0E-03, θc=1.0E-04), 3.90E-02 unavailable, 95% CI ⊆ ±18.9%)
# SimResult(SimConfig(m=10, g=3, p=4.0E-01, θd=1.0E-03, θc=1.0E-04), 6.91E-01 unavailable, 95% CI ⊆ ±2.4%)
# SimResult(SimConfig(m=10, g=4, p=1.0E-02, θd=1.0E-03, θc=1.0E-04), 0.00E+00 unavailable, 95% CI ⊆ ±NaN%)
# SimResult(SimConfig(m=10, g=4, p=1.0E-01, θd=1.0E-03, θc=1.0E-04), 0.00E+00 unavailable, 95% CI ⊆ ±NaN%)
# SimResult(SimConfig(m=10, g=4, p=2.0E-01, θd=1.0E-03, θc=1.0E-04), 3.33E-04 unavailable, 95% CI ⊆ ±367.1%)
# SimResult(SimConfig(m=10, g=4, p=4.0E-01, θd=1.0E-03, θc=1.0E-04), 6.31E-01 unavailable, 95% CI ⊆ ±2.8%)
# SimResult(SimConfig(m=10, g=5, p=1.0E-02, θd=1.0E-03, θc=1.0E-04), 0.00E+00 unavailable, 95% CI ⊆ ±NaN%)
# SimResult(SimConfig(m=10, g=5, p=1.0E-01, θd=1.0E-03, θc=1.0E-04), 0.00E+00 unavailable, 95% CI ⊆ ±NaN%)
# SimResult(SimConfig(m=10, g=5, p=2.0E-01, θd=1.0E-03, θc=1.0E-04), 3.33E-04 unavailable, 95% CI ⊆ ±367.1%)
# SimResult(SimConfig(m=10, g=5, p=4.0E-01, θd=1.0E-03, θc=1.0E-04), 5.16E-01 unavailable, 95% CI ⊆ ±3.5%)
