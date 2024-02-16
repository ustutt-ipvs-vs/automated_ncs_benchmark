# Ja: println(mean([J1,J2,J3,J4]))
# Jv: println(var([J1,J2,J3,J4]))

using Logging
using Statistics
using JLD

Ts = 10e-3
# model = :cintegrator
# model = :powersystem
model = :pendulum

seed = 1
tsim = 600
shareOV = true
hold = false

# variant = :default
# variant = :allprop
variant = :shareIV

# est_cmp = :default
est_cmp = :optsc
# est_cmp = :opt

if isdefined(Main, :ReplicationSim)
	if est_cmp == :optsc
		protocol = Symbol("replic_",variant,"_optsc")
		ReplicationSim.isless(a::Estimate,b::Estimate) = isless((a.vp,a.kp,-a.loss),(b.vp,b.kp,-b.loss))
	elseif est_cmp == :opt
		protocol = Symbol("replic_",variant,"_opt")
		ReplicationSim.isless(a::Estimate,b::Estimate) = isless((a.vp,-a.loss),(b.vp,-b.loss))
	else # assume est_cmp == :default
		protocol = Symbol("replic_",variant)
		ReplicationSim.isless(a::Estimate,b::Estimate) = isless((a.vp,a.kp),(b.vp,b.kp))
	end
else
	protocol = :quarts
end

include("params.jl") # defines `plant`, `sys`, and `baseconf` and a `config` for warmup

ploss_range = [1e-5]#[1e-8,1e-7,1e-6,1e-5]
pcrash_range = [1e-2]#[10^-3.5,10^-3,10^-2.5,10^-2]
params = Iterators.product(ploss_range, pcrash_range)

filename = string(protocol,"-seed",seed,"-pl",(ploss_range[1],ploss_range[end]),"-pc",(pcrash_range[1],pcrash_range[end]),".jld")

println("Running simulations for")
println(filename)

if isdefined(Main, :Distributed) && nworkers() > 1

	results = pmap(params; on_error=identity) do (ploss,pcrash)
	    config = SimConfig(baseconf,(p=ploss,θd=ploss,θc=pcrash,R=0.5))
	    # warmup
	    runsim(config,Twarmup(config),seed,Tmin=10,Tmax=60,share_OV=shareOV,hold=hold,logfields=(:p,:θc),variant=variant)
	    println("=== warmup completed ===")
	    # simulation
	    runsim(config,tsim,seed,Tmin=10,Tmax=60,share_OV=shareOV,hold=hold,logfields=(:p,:θc),variant=variant)
	end

else

	# warmup
	runsim(config,Twarmup(config),seed,Tmin=10,Tmax=60,share_OV=shareOV,hold=hold,logfields=(:p,:θc),variant=variant)
    println("=== warmup completed ===")

	results = Vector{SimResult}(undef, length(params))

	for (i,(ploss,pcrash)) in enumerate(params)
		config = SimConfig(baseconf,(p=ploss,θd=ploss,θc=pcrash,R=0.5))
		# simulation
		results[i] = runsim(config,tsim,seed,Tmin=10,Tmax=60,share_OV=shareOV,hold=hold,logfields=(:p,:θc),variant=variant)
	end

end

try
	save(filename, "seed", seed, "results", results)
catch e
	@error "Could not write results to file!"
	showerror(stderr, e, catch_backtrace())
end


# results = Dict{Float64,SimResult}()

# for loss_probability in [0.0, 0.005, 0.01, 0.02, 0.04] # 0.01:0.001:0.02 # 0.05:0.05:0.2
# 	config = SimConfig(baseconf,(p=loss_probability,))
# 	results[loss_probability] = runsim(config,tsim,seed,Tmin=2,Tmax=2,share_OV=shareOV,hold=hold,logfields=(:p,),variant=variant)
# 	println(results[loss_probability])
# end

# # for crash_probability in 0.05:0.05:0.25 # 0.02:0.005:0.04
# # 	config = SimConfig(baseconf,(θc=crash_probability,))
# # 	results[crash_probability] = runsim(config,tsim,seed,Tmin=2,Tmax=2,share_OV=shareOV,hold=hold,logfields=(:θc,))
# # end

# # crash_probability = 0.01
# # loss_probability = 0.05
# # for crash_duration in (2:2:6)*Ts
# # 	config = SimConfig(baseconf,(θc=crash_probability,R=crash_duration,p=loss_probability))
# # 	results[crash_duration] = runsim(config,tsim,seed,Tmin=2,Tmax=2,share_OV=shareOV,hold=hold,logfields=(:θc,:R,:p))
# # end

# J = sort([ p => mean(r.cost) for (p,r) in pairs(results) ])
# u = sort([ p => r.unavailability for (p,r) in pairs(results) ])

# println("u = ",last.(u))
# println("J = ",last.(J))

# with_logger(ConsoleLogger(stdout,Logging.Info)) do
# # with_logger(ConsoleLogger(stdout,Logging.Debug)) do
#	config = SimConfig(baseconf,(p=0e-5,θd=0e-3,θc=1e-3,R=1.0))
# 	redirect_stderr(open(string(model,"_seed",seed,"_tsim",tsim,"_p",config.p,"_td",config.θd,"_tc",config.θc,".log"),"w")) do
# 		global result = runsim(config,tsim,seed,Tmin=2,Tmax=2,share_OV=shareOV,hold=hold,variant=variant)
# 		flush(stderr)
# 	end
# 	show(result)
# end

# result = runsim(config,tsim,seed,Tmin=2,Tmax=2,share_OV=shareOV,hold=hold,variant=variant)

## Experiments

#####################################
# loss_probability in 0.05:0.05:0.25
# share_OV = true
# hold = true
# tsim = 100

# ReplicationSim.isless(a::Estimate,b::Estimate) = isless((a.vp,a.kp),(b.vp,b.kp))
# seed=10: J1 = [0.994758, 1.31174, 1.78123, 2.53369, 3.89156]

# ReplicationSim.isless(a::Estimate,b::Estimate) = isless((a.vp,a.kp,-a.loss),(b.vp,b.kp,-b.loss))
# seed=10: J2 = [0.969807, 1.23911, 1.47652, 1.94016, 2.91416]

# ReplicationSim.isless(a::Estimate,b::Estimate) = isless((a.vp,-a.loss),(b.vp,-b.loss))
# seed=10: J3 = [0.968045, 1.21872, 1.45384, 1.86836, 2.80604]


#####################################
# loss_probability in 0.05:0.05:0.25
# share_OV = false
# hold = true
# tsim = 100

# ReplicationSim.isless(a::Estimate,b::Estimate) = isless((a.vp,a.kp),(b.vp,b.kp))
# seed=10: J1 = [0.994758, 1.31174, 2.57753, 4.26717, 875.825]

# ReplicationSim.isless(a::Estimate,b::Estimate) = isless((a.vp,a.kp,-a.loss),(b.vp,b.kp,-b.loss))
# seed=10: J2 = [0.969807, 1.23911, 2.02036, 3.47397, 208.111]

# ReplicationSim.isless(a::Estimate,b::Estimate) = isless((a.vp,-a.loss),(b.vp,-b.loss))
# seed=10: J3 = [0.968045, 1.2187, 2.02418, 2.97937, 203.333]


#####################################
# loss_probability in 0.05:0.05:0.25
# share_OV = true
# hold = false
# tsim = 100

# ReplicationSim.isless(a::Estimate,b::Estimate) = isless((a.vp,a.kp),(b.vp,b.kp))
# seed=10: J1 = [0.927634, 1.31131, 1.80353, 2.52548, 3.59325]

# ReplicationSim.isless(a::Estimate,b::Estimate) = isless((a.vp,a.kp,-a.loss),(b.vp,b.kp,-b.loss))
# seed=10: J2 = [0.898205, 1.23867, 1.50157, 1.93194, 2.62484]

# ReplicationSim.isless(a::Estimate,b::Estimate) = isless((a.vp,-a.loss),(b.vp,-b.loss))
# seed=10: J3 = [0.897526, 1.21829, 1.47831, 1.86229, 2.50353]


#####################################
# loss_probability in 0.05:0.05:0.25
# share_OV = false
# hold = false
# tsim = 100

# ReplicationSim.isless(a::Estimate,b::Estimate) = isless((a.vp,a.kp),(b.vp,b.kp))
# seed=10: J1 = [0.927634, 1.31131, 2.60703, 3.62654, 310.571]

# ReplicationSim.isless(a::Estimate,b::Estimate) = isless((a.vp,a.kp,-a.loss),(b.vp,b.kp,-b.loss))
# seed=10: J2 = [0.898205, 1.23867, 2.05497, 2.51096, 74.354]

# ReplicationSim.isless(a::Estimate,b::Estimate) = isless((a.vp,-a.loss),(b.vp,-b.loss))
# seed=10: J3 = [0.897526, 1.21827, 2.05803, 2.47897, 24.3759]


######################################
# crash_probability = 0.01
# loss_probability = 0.05
# crash_duration in (2:2:6)*Ts
# share_OV = true
# tsim = 100

# ReplicationSim.isless(a::Estimate,b::Estimate) = isless((a.vp,a.kp),(b.vp,b.kp))
# seed=10: J1 = [1.02977, 1.14675, 1.18831]

# ReplicationSim.isless(a::Estimate,b::Estimate) = isless((a.vp,a.kp,-a.loss),(b.vp,b.kp,-b.loss))
# seed=10: J2 = [0.992859, 1.1166, 1.15067]

# ReplicationSim.isless(a::Estimate,b::Estimate) = isless((a.vp,-a.loss),(b.vp,-b.loss))
# seed=10: J3 = [0.992662, 1.11451, 1.15013]


######################################
# crash_probability = 0.01
# loss_probability = 0.05
# crash_duration in (2:2:6)*Ts
# share_OV = false
# tsim = 100

# ReplicationSim.isless(a::Estimate,b::Estimate) = isless((a.vp,a.kp),(b.vp,b.kp))
# seed=10: J1 = [2.77458, 1.20509, 1.29424]

# ReplicationSim.isless(a::Estimate,b::Estimate) = isless((a.vp,a.kp,-a.loss),(b.vp,b.kp,-b.loss))
# seed=10: J2 = [2.73826, 1.17662, 1.25337]

# ReplicationSim.isless(a::Estimate,b::Estimate) = isless((a.vp,-a.loss),(b.vp,-b.loss))
# seed=10: J3 = [2.73812, 1.17321, 1.25283]

