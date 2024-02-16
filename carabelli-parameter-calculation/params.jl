using LinearAlgebra

if model == :pendulum
	plant = InvertedPendulum(Ts, 1.0; C=Diagonal(ones(4))[[1,3],:], W=Diagonal(1e-1*ones(4)), V=Diagonal(1e-1*ones(2))) |> PlantModel
elseif model == :powersystem || model == :cintegrator
	if model == :powersystem
		include("models/powersystem-matrices.jl")
		W = 1e-0*Matrix{Float64}(I,size(A))
	elseif model == :cintegrator
		## continous-time n-integrator
		p = 10 # number of sensors
		h = 5 # number of hidden states
		A = diagm(1 => ones(p+h-1))
		B = Matrix{Float64}(I,size(A))[:,end:end]
		C = Matrix{Float64}(I,size(A))[1:p,:]
		W = 1e-3*Matrix{Float64}(I,size(A))
	end
	V = 1e-3*Matrix{Float64}(I,size(C,1),size(C,1))
	Q = inv(size(A,1))*Matrix{Float64}(I,size(A))
	H = zeros(size(B))
	R = 0.1*Matrix{Float64}(I,size(B,2),size(B,2))
	plant = PlantModel(Ts,c2d(A,copy(B),W,Q,H,R,Ts)...,C,V)
else
	if model == :toymodel
		## toy model (1×5)
		# A = Float64[0 1 0 0 0; 0 0 1 0 0; 0 0 0 1 0; 0 0 0 0 1; 2 2 2 2 2]/2
		A = Float64[0 1 0 0 0; 0 0 1 0 0; 0 0 0 1 0; 0 0 0 0 1; 2 -2 2 -2 2]/1.1
		B = Matrix{Float64}(I,size(A))[:,end:end]
		C = Matrix{Float64}(I,size(A))
	elseif model == :dintegrator
		## discrete-time n-integrator
		n = 7
		A = diagm(0 => ones(n), 1 => ones(n-1))
		B = Matrix{Float64}(I,size(A))[:,end:end]
		C = Matrix{Float64}(I,size(A))[1:end-1,:]
	elseif model == :batchreactor
		## batch reactor model
		## Walsh et al. "Stability Analysis of Networked Control Systems", IEEE Trans. Control Systems Technology, vol. 10(3), May 2002
		A = Float64[
		   1.38   -0.2077  6.715  -5.676;
		  -0.5814 -4.29    0.0     0.675;
		   1.067   4.273  -6.654   5.893;
		   0.048   4.273   1.343  -2.104
		]
		B = Float64[
		   0.0    0.0;
		   5.679  0.0;
		   1.136 -3.146;
		   1.136  0.0
		]
		# C = Matrix{Float64}(I,size(A))
		# C = Float64[
		#   1  0  1 -1;
		#   0  1  0  0
		# ]
		C = Matrix{Float64}(I,size(A))[[1],:]
	else
		@error "Unknown model" model
	end

	W = 1e-3*Matrix{Float64}(I,size(A))
	V = 1e-6*Matrix{Float64}(I,size(C,1),size(C,1))
	Q = 1.0*Matrix{Float64}(I,size(A))
	H = zeros(size(B))
	R = 0.1*Matrix{Float64}(I,size(B,2),size(B,2))
	plant = PlantModel(Ts,A,B,W,Q,H,R,C,V)
end

sys = ControlSystem(plant)

#                          n,       m, g,       h,  T,     δn,    p,   θd,   θc,    τ,   R, sys
baseconf = SimConfig(plant.n, plant.p, 3, plant.m, Ts, 0.5e-3, 1e-2, 1e-3, 1e-4, 8e-3, 1.0, sys)
# baseconf = SimConfig(plant.n, plant.p, 3, plant.m, Ts, 0.5e-3, 1e-2, 1e-3, 1e-4, 8e-3,  Ts, sys)
# config = SimConfig(plant.n, plant.p, 3, plant.m, Ts, 0.5e-3,  0.5, 1e-3, 1e-4, 8e-3, 1.0, sys)

config = baseconf
