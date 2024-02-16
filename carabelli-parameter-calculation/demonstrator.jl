(@__DIR__) in LOAD_PATH || push!(LOAD_PATH, @__DIR__)

using LQRUtils
using PlantModels
using LinearAlgebra

periodMillis = 50
Ts = periodMillis/1000

# max_pos = 0.2
# max_ang = deg2rad(10)

# q = [ 1, 0.01, (max_pos/max_ang)^2, 0.01 ]
q = [ 1, 0.01, 1.5, 0.01 ]
r = 0.03

v = [ 1.5e-4, 1.5e-4, 2.5e-3, 2.5e-3 ]
w = [ 1e-3, 1e-3, 1e-1, 2.5 ]

plant = InvertedPendulum(Ts, 0.6;
	C=Diagonal(ones(4))[1:3,:], V=Diagonal(v[1:3]),
	# C=Diagonal(ones(4))[[1,3],:], V=Diagonal(v[[1,3]]),
	Q=Diagonal(q), R=r,
	W=Diagonal(w)
) |> PlantModel

sys = ControlSystem(plant)

K = - sys.K
# [-1.3707775758617196 -2.8003576442097353 31.136433243982783 8.263746547485932]

L = dkalman(plant.A, plant.C, plant.W, plant.V)

Lx = round.((I-L*plant.C)*plant.A, digits=12)
Lu = round.((I-L*plant.C)*plant.B, digits=12)
Ly = round.(L, digits=12)

println("constexpr unsigned balancePeriod = ",periodMillis,";")
println();

Kname = ["xc","vc","xp","vp"]
# Ksuffix = ["",""," * MSTEP_PER_METER * RAD_PER_ESTEP"," * MSTEP_PER_METER * RAD_PER_ESTEP"]
for i in eachindex(K)
	# println("constexpr float K",Kname[i]," = ",K[i],Ksuffix[i],";")
	println("constexpr float K",Kname[i]," = ",K[i],";")
end

println();

function printconsts(A, name)
	ind = ( count(1 .== size(A)) == ndims(A)-1 ) ? LinearIndices(A) : CartesianIndices(A)
	for i in ind
		isapprox(0,A[i],atol=1e-10) || println("constexpr float ",name,Tuple(i)...," = ",A[i],";")
	end
end

printconsts(Lx, "Lx")
printconsts(Lu, "Lu")
printconsts(Ly, "Ly")

# function printCarray(A, name)
# 	println("constexpr float ",name,"[",size(A,1),"][",size(A,2),"] = {")
# 	for i in 1:size(A,1)
# 		print("  { ")
# 		for j in 1:size(A,2)
# 			print(A[i,j], j==size(A,2) ? "" : ", ")
# 		end
# 		println(" }", i==size(A,1) ? "" : ",")
# 	end
# 	println("};")
# end

println()
Acl = plant.A + plant.B * K

function evsort(M)
	e = eigen(M)
	ord = sortperm( vec( sum( abs.(e.vectors) .* collect(axes(e.vectors,1)), dims=1 ) ) )
	e.values[ord]
end

@show abs.(evsort( Acl^(1/Ts) ))
@show abs.(evsort( Lx^(1/Ts) ))
nothing
