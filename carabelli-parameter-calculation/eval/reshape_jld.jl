using JLD
using Distibutions
include("../replication-sim/load-sim.jl")
include("../quarts-sim/load-sim.jl")

jld_src = ...
jld_dst = ...

function reshape_jld(jld_src, jld_dst)
	parameters = (:p,:θc)

	seed = load(jld_src, "seed")
	results = load(jld_src, "results")

	parameter_values = [ getfield.((r.config,),parameters) for r in results ]
	dims = Tuple( length(unique(getindex.(parameter_values,i))) for i in 1:length(parameters) )

	results = reshape(results, dims)
	parameter_values = reshape(parameter_values, dims)

	@save jld_dst seed parameters parameter_values results

	results
end
