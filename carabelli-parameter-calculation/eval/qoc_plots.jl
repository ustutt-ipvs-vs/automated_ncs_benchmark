include("../replication-sim/load-sim.jl")
include("../quarts-sim/load-sim.jl")

JLDPREFIX = "../../legion/results/"

results = Dict{Tuple{Symbol,Int},Dict{String,Any}}()
for seed in 1:4
    results[(:replicdef,seed)] = load(joinpath(JLDPREFIX,"replic-default_results_seed$(seed).jld"))
    results[(:replicopt,seed)] = load(joinpath(JLDPREFIX,"replic-shareIV-optsc_results_seed$(seed).jld"))
    results[(:quarts,seed)] = load(joinpath(JLDPREFIX,"quarts_results_seed$(seed).jld"))
end

J = Dict( i => [ mean(r.cost) for r in v["results"] ][1:4,1:4] for (i,v) in results )
u = Dict( i => [ r.unavailability for r in v["results"] ][1:4,1:4] for (i,v) in results )

param = Dict( i => v["parameter_values"][1:4,1:4] for (i,v) in results )
pcommon = unique(values(param))
@assert length(pcommon) == 1

p = first.(pcommon[1])
tc = last.(pcommon[1])

Jsorted(x) = last.(sort(collect( (i,J[(proto,i)]) for (proto,i) in keys(J) if proto == x ), by=first))

Jreplicdef = Jsorted(:replicdef)
Jreplicopt = Jsorted(:replicopt)
Jquarts = Jsorted(:quarts)

p_plt = p[:,1]
tc_plt = tc[1,:]

Jloss_opt_plt = mean( a ./ b for (a,b) in zip(Jreplicdef, Jreplicopt) )'
Jloss_quarts_plt = mean( a ./ b for (a,b) in zip(Jquarts, Jreplicopt) )'

plt_loss_opt = plot(tc_plt, Jloss_opt_plt, xscale=:log10, yscale=:identity, lw=3, label=[string("loss probability ",p) for p in p_plt], xlabel="crash failure probability", ylabel="cost ratio (basic/optimized)", legend=:topleft, xlim=(10^-3.7,10^-1.8), ylim=1 .+ (-0.0001,0.0001) )

plt_loss_quarts = plot(tc_plt, Jloss_quarts_plt, scale=:log10, lw=3, label=[string("loss probability ",p) for p in p_plt], xlabel="crash failure probability", ylabel="cost ratio (Quarts/optimized)", legend=:topleft, xlim=(10^-3.7,10^-1.8), ylim=(1e-1,2e3) )
