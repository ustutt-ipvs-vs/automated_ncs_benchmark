p = collect(0.05:0.05:0.25)
i_method = [1, 2, 3]
n_method = [:original, :state_consistent_optimization, :full_optimization]
i_shareOV = [true, false]
i_hold = [true, false]

# J = fill(0.0,length(i_p),length(i_method),length(i_shareOV),length(i_hold))

Jrep = Dict{Int,Array{Float64,4}}() # replications of results by seed

# result values for 
Jrep[10] = fill(NaN,length(p),length(i_method),length(i_shareOV),length(i_hold))
# share_OV = true, hold = true
Jrep[10][:,1,1,1] = [0.994758, 1.31174, 1.78123, 2.53369, 3.89156]
Jrep[10][:,2,1,1] = [0.969807, 1.23911, 1.47652, 1.94016, 2.91416]
Jrep[10][:,3,1,1] = [0.968045, 1.21872, 1.45384, 1.86836, 2.80604]
# share_OV = false, hold = true
Jrep[10][:,1,2,1] = [0.994758, 1.31174, 2.57753, 4.26717, 875.825]
Jrep[10][:,2,2,1] = [0.969807, 1.23911, 2.02036, 3.47397, 208.111]
Jrep[10][:,3,2,1] = [0.968045, 1.2187, 2.02418, 2.97937, 203.333]
# share_OV = true, hold = false
Jrep[10][:,1,1,2] = [0.927634, 1.31131, 1.80353, 2.52548, 3.59325]
Jrep[10][:,2,1,2] = [0.898205, 1.23867, 1.50157, 1.93194, 2.62484]
Jrep[10][:,3,1,2] = [0.897526, 1.21829, 1.47831, 1.86229, 2.50353]
# share_OV = false, hold = false
Jrep[10][:,1,2,2] = [0.927634, 1.31131, 2.60703, 3.62654, 310.571]
Jrep[10][:,2,2,2] = [0.898205, 1.23867, 2.05497, 2.51096, 74.354]
Jrep[10][:,3,2,2] = [0.897526, 1.21827, 2.05803, 2.47897, 24.3759]

println("Loaded simulation results for seed values ", keys(Jrep))
