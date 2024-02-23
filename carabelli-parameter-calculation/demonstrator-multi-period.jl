(@__DIR__) in LOAD_PATH || push!(LOAD_PATH, @__DIR__)

using LQRUtils
using PlantModels
using LinearAlgebra

# Open the file for writing
const OUTPUT_FILE = "../sender-receiver-teensy/pendulum_receiver1/CarabelliParameters.h"
global IO = open(OUTPUT_FILE, "w")

q = [ 1, 0.01, 1.5, 0.01 ]
r = 0.03

v = [ 1.5e-4, 1.5e-4, 2.5e-3, 2.5e-3 ]
w = [ 1e-3, 1e-3, 1e-1, 2.5 ]

# Which sampling periods to generate parameters for:
minSamplingPeriod = 10
maxSamplingPeriod = 200
samplingPeriodStepSize = 2

function printconsts(A, name)
	ind = ( count(1 .== size(A)) == ndims(A)-1 ) ? LinearIndices(A) : CartesianIndices(A)
	for i in ind
		isapprox(0,A[i],atol=1e-10) || println(IO, "      ",name,Tuple(i)...," = ",A[i],";")
	end
end

function printVariableInitializationBlock()
	println(IO, "// Initialize with parameters for T = 50 ms")
	println(IO, "float Kxc = 4.371647840086138;")
	println(IO, "float Kvc = 5.153527889236189;")
	println(IO, "float Kxp = -39.38984870058722;")
	println(IO, "float Kvp = -10.434155921005745;")
	println(IO)
	println(IO, "float Lx11 = 0.564251064459;")
	println(IO, "float Lx21 = -0.014108600623;")
	println(IO, "float Lx12 = -0.007583061891;")
	println(IO, "float Lx22 = 0.565554280146;")
	println(IO, "float Lx33 = 0.010356446081;")
	println(IO, "float Lx43 = -3.648011475727;")
	println(IO, "float Lx34 = 0.000511859164;")
	println(IO, "float Lx44 = 0.802436276753;")
	println(IO, "float Lu1 = -0.001084466925;")
	println(IO, "float Lu2 = 0.028295349758;")
	println(IO, "float Lu3 = 1.8227504e-5;")
	println(IO, "float Lu4 = 0.064185457789;")
	println(IO, "float Ly11 = 0.435748935541;")
	println(IO, "float Ly21 = 0.014108600623;")
	println(IO, "float Ly12 = 0.035795615114;")
	println(IO, "float Ly22 = 0.433740289823;")
	println(IO, "float Ly33 = 0.989822343862;")
	println(IO, "float Ly43 = 4.27759379409;")
	println(IO)
end

function calcParametersForSamplingperiod(periodMillis)
	Ts = periodMillis/1000
	plant = InvertedPendulum(Ts, 0.6;
		C=Diagonal(ones(4))[1:3,:], V=Diagonal(v[1:3]),
		Q=Diagonal(q), R=r,
		W=Diagonal(w)
	) |> PlantModel

	sys = ControlSystem(plant)

	K = - sys.K

	L = dkalman(plant.A, plant.C, plant.W, plant.V)

	Lx = round.((I-L*plant.C)*plant.A, digits=12)
	Lu = round.((I-L*plant.C)*plant.B, digits=12)
	Ly = round.(L, digits=12)

	println(IO, "    case ",periodMillis,":")

	Kname = ["xc","vc","xp","vp"]
	for i in eachindex(K)
		println(IO, "      K",Kname[i]," = ",K[i],";")
	end

	println(IO)

	printconsts(Lx, "Lx")
	printconsts(Lu, "Lu")
	printconsts(Ly, "Ly")

	println(IO, "      break;")
	println(IO)
end

function printFunctionBeginning(minVal, maxVal, stepSize)
	header = """
	void updateCarabelliParameters(unsigned samplingPeriod){
		const unsigned int minVal = $minVal;
		const unsigned int maxVal = $maxVal;
		const unsigned int stepSize = $stepSize;
		const unsigned int remainder = samplingPeriod % stepSize;
		if (remainder != 0) {
			samplingPeriod += stepSize - remainder; // Round up to the next multiple of stepSize
		}

		// Ensure samplingPeriod is within the range [minVal, maxVal]
		samplingPeriod = min(maxVal, max(minVal, samplingPeriod));

		switch(samplingPeriod){
	"""
	println(IO, header)
end


println(IO, "#ifndef CARABELLI_PARAMETERS_H")
println(IO, "#define CARABELLI_PARAMETERS_H")
println(IO, "// This file has been generated by the script in ../../carabelli-parameter-calculation/demonstrator-multi-period.jl")
println(IO)
printVariableInitializationBlock()
printFunctionBeginning(minSamplingPeriod, maxSamplingPeriod, samplingPeriodStepSize)
for i in minSamplingPeriod:samplingPeriodStepSize:maxSamplingPeriod
	println("Calculating parameters for T=", i, "ms")
	calcParametersForSamplingperiod(i)
end
println(IO, "  }")
println(IO, "}")
println(IO)
println(IO, "#endif")

println("Done. Output written to file ", OUTPUT_FILE)
nothing
