include("../replication-sim/load-sim.jl")
include("../quarts-sim/load-sim.jl")
using JLD, Distributions

### EXPERIMENTS ###

# paper var:           nx, ny, N, nu,    Ts,      δ,    p,   θd,   θc,   (τ),   R
# code var:             n,  m, g,  h,     T,     δn,    p,   θd,   θc,     τ,   R
baseconfig = SimConfig( 2, 10, 3,  1, 20e-3, 0.5e-3, 1e-2, 1e-3, 1e-4, 1e-10, 1.0)

# uniform length for p and tc
prunsim(baseconfig, params, until::Real, seed) = pmap(params; on_error=identity) do p
    config = SimConfig(baseconfig, p)
    result = runsim(config, until, seed; Tmin=10, Tmax=60, quiet=false, logfields=(:p,:θc))
    result
end
prunsim(baseconfig, params, until, seed, failseed) = pmap(params; on_error=identity) do p
    config = SimConfig(baseconfig, p)
    result = runsim(config, until, seed, failseed; Tmin=10, Tmax=60, quiet=false, logfields=(:p,:θc))
    result
end
prunsim(baseconfig, fill(NamedTuple(),nworkers()), 0.1, 1) # warmup

# varying length for T
prunsim(baseconfig, params, untils::AbstractVector, seed) = pmap(zip(untils,params); on_error=identity) do (until,p)
    config = SimConfig(baseconfig, p)
    result = runsim(config, until, seed; Tmin=10, Tmax=60, quiet=false, logfields=(:p,:θc,:T))
    result
end
prunsim(baseconfig, fill(NamedTuple(),nworkers()), fill(0.1,nworkers()), 1) # warmup

# params = merge.([()],[(),()])

params = [(p=0.0001,),(p=0.0003,),(p=0.001,),(p=0.003,),(p=0.01,),(p=0.03,),(p=0.1,)]
sims = prunsim(baseconfig, params, 300*60, 1)

params = [(θc=0.0001,),(θc=0.0003,),(θc=0.001,),(θc=0.003,),(θc=0.01,),(θc=0.03,)]#,(θc=0.1,)]
params = [(θc=0.0001,),(θc=0.0003,),(θc=0.001,)]
params = [(θc=0.003,),(θc=0.01,),(θc=0.03,),(θc=0.1,)]
sims = prunsim(baseconfig, params, 30*60, 1)

params = [(T=5e-3,),(T=10e-3,),(T=15e-3,),(T=20e-3,),(T=25e-3,)]
params = [(T=1e-3,),(T=2e-3,),(T=3e-3,)]
params = [(T=4e-3,),(T=6e-3,)]
untils = [p.T/0.02*240*60 for p in params]
sims = prunsim(baseconfig, params, untils, 1)


### RESULTS ###

cd("study-p")
# sims_replic = load("replic_t240_seed2_g3_p0001-03_tc00001.jld","sims")
sims_replic1 = load("replic_t300_seed1_g3_p00001-01_tc00001.jld","sims")
sims_replic2 = load("replic_t300_seed2_g3_p00001-01_tc00001.jld","sims")
sims_replicn = load("replic_t300_seedn_g3_p00001-01_tc00001.jld","sims")
# sims_quarts = load("quarts_t240_seed2_g3_p0001-03_tc00001.jld","sims")
sims_quarts1 = load("quarts_t300_seed1_g3_p00001-01_tc00001.jld","sims")
sims_quarts2 = load("quarts_t300_seed2_g3_p00001-01_tc00001.jld","sims")
ps = [s.config.p for s in sims_replic1]
### unreliability
# [(s.config.p, s.unavailability) for s in sims_replic]
# [(s.config.p, s.unavailability) for s in sims_quarts]
u_replic = mean([getfield.(sims,:unavailability) for sims in [sims_replic1,sims_replic2,sims_replicn]])
u_quarts = mean([getfield.(sims,:unavailability) for sims in [sims_quarts1,sims_quarts2]])
collect(zip(ps,u_replic))
collect(zip(ps,u_quarts))
### latency [ms]
# [(s.config.p, mean(filter(!isnan,s.delay))*1e3 ) for s in sims_replic]
# [(s.config.p, mean(filter(!isnan,s.delay))*1e3 ) for s in sims_quarts]
l_replic = mean([[mean(filter(!isnan,s.delay))*1e3 for s in sim] for sim in [sims_replic1,sims_replic2,sims_replicn]])
l_quarts = mean([[mean(filter(!isnan,s.delay))*1e3 for s in sim] for sim in [sims_quarts1,sims_quarts2]])
collect(zip(ps,l_replic))
collect(zip(ps,l_quarts))
### message cost [1/T]
# [(s.config.p, ReplicationSim.messages_per_period(s) ) for s in sims_replic]
# [(s.config.p, QuartsSim.messages_per_period(s) ) for s in sims_quarts]
m_replic = mean([[ReplicationSim.messages_per_period(s) for s in sim] for sim in [sims_replic1,sims_replic2,sims_replicn]])
m_quarts = mean([[QuartsSim.messages_per_period(s) for s in sim] for sim in [sims_quarts1,sims_quarts2]])
collect(zip(ps,m_replic))
collect(zip(ps,m_quarts))
cd("..")

cd("study-tc")
simss_replic_01 = load.(["replic_t30_seed$(s)_g3_p001_tc0003-01.jld" for s in (3,4,6,7)],["sims"])
simss_replic_00001 = load.(["replic_t60_seed$(s)_g3_p001_tc00001-0001.jld" for s in (2,3,4,5)],["sims"])
# sims_replic_00001 = load("replic_t180_seed3_g3_p001_tc00001-0001.jld","sims")
sims_quarts2 = load("quarts_t180_seed2_g3_p001_tc00001-01.jld","sims")
sims_quarts3 = load("quarts_t180_seed3_g3_p001_tc00001-01.jld","sims")
tcs = [s.config.θc for s in sims_quarts2]
### unreliability
u_quarts = mean([getfield.(sims,:unavailability) for sims in [sims_quarts2,sims_quarts3]])
collect(zip(tcs,u_quarts))
u_replic = vcat( mean([getfield.(sims,:unavailability) for sims in simss_replic_00001]), mean([getfield.(sims,:unavailability) for sims in simss_replic_01]) )
collect(zip(tcs,u_replic))
### latency [ms]
l_quarts = mean([[mean(filter(!isnan,s.delay))*1e3 for s in sim] for sim in [sims_quarts2,sims_quarts3]])
collect(zip(tcs,l_quarts))
l_replic = vcat( mean([[mean(filter(!isnan,s.delay))*1e3 for s in sim] for sim in simss_replic_00001]), mean([[mean(filter(!isnan,s.delay))*1e3 for s in sim] for sim in simss_replic_01]) )
collect(zip(tcs,l_replic))
### message cost [1/T]
m_quarts = mean([[QuartsSim.messages_per_period(s) for s in sim] for sim in [sims_quarts2,sims_quarts3]])
collect(zip(tcs,m_quarts))
m_replic = vcat( mean([[ReplicationSim.messages_per_period(s) for s in sim] for sim in simss_replic_00001]), mean([[ReplicationSim.messages_per_period(s) for s in sim] for sim in simss_replic_01]) )
collect(zip(tcs,m_replic))
cd("..")

cd("study-T")
sims_quarts = load("quarts_seed1_p001_tc00001_T1-25.jld","sims")
sims_replic = load("replic_seed1_p001_tc00001_T1-25.jld","sims")
### unreliability
[(s.config.T, s.unavailability) for s in sims_replic]
[(s.config.T, s.unavailability) for s in sims_quarts]
### latency [ms]
# [(s.config.T, mean(filter(!isnan,s.delay))*1e3 ) for s in sims_replic]
# [(s.config.T, mean(filter(!isnan,s.delay))*1e3 ) for s in sims_quarts]
### message cost [1/T]
# [(s.config.T, ReplicationSim.messages_per_period(s) ) for s in sims_replic]
# [(s.config.T, QuartsSim.messages_per_period(s) ) for s in sims_quarts]
cd("..")
