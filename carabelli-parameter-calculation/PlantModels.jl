# julia 1.1.0
module PlantModels

using LQRUtils, LinearAlgebra

export ControlSystem, PlantModel, Simple1D, Simple2D, InvertedPendulum

"""
    PlantModel(Ts, A, B, W, Q, H, R)

Composite type for discrete-time LTI plant models

``xₖ₊₁ = Axₖ + Buₖ + wₖ,  wₖ ~ W``

with sampling period `Ts` and associated LQR stage cost

``J = xᵀQx + 2xᵀHu + uᵀRu``

Constructor checks for appropriate matrix dimensions.
"""
struct PlantModel
	# SAMPLING TIME
	Ts::Float64
	# SYSTEM MATRICES
	A::Matrix{Float64} # n×n state transition
	B::Matrix{Float64} # n×m input
	W::Matrix{Float64} # n×n state noise
	C::Matrix{Float64} # pxn output
	V::Matrix{Float64} # pxp measurement noise
	# LQR COST COEFFICIENTS
	Q::Matrix{Float64} # n×n state cost
	H::Matrix{Float64} # n×m cross-cost
	R::Matrix{Float64} # m×m input cost
	# SYSTEM DIMENSIONS
	n::Int
	m::Int
	p::Int

	function PlantModel(Ts::Float64,
		A::Matrix{Float64}, B::Matrix{Float64}, W::Matrix{Float64},
		Q::Matrix{Float64}, H::Matrix{Float64}, R::Matrix{Float64},
		C::Matrix{Float64}=Matrix{Float64}(I,size(A)),
		V::Matrix{Float64}=zeros(size(A)))
		n, m, p = sysdims_full(A,B,C,W,V,Q,H,R)
		new(Ts, A, B, W, C, V, Q, H, R, n, m, p)
	end
end

### DEFINE SYSTEM TYPES ###
abstract type ModelType end
include("models/Simple1D.jl")
include("models/Simple2D.jl")
include("models/InvertedPendulum.jl")

### OVERLOAD METHODS FROM LQRUtils FOR PlantModel AND ModelType ARGUMENTS ###
LQRUtils.driccati(P, sys::PlantModel) = driccati(P, sys.A, sys.B, sys.Q, sys.H, sys.R)
LQRUtils.driccati(P, systype::T) where T<:ModelType = dricatti(P, PlantModel(systype))
LQRUtils.dare(sys::PlantModel) = dare(sys.A, sys.B, sys.Q, sys.H, sys.R)
LQRUtils.dare(systype::T) where T<:ModelType = dare(PlantModel(systype))
LQRUtils.dlqrp(sys::PlantModel) = dlqrp(sys.A, sys.B, sys.Q, sys.H, sys.R)
LQRUtils.dlqrp(systype::T) where T<:ModelType = dlqrp(PlantModel(systype))
LQRUtils.dlqr(sys::PlantModel) = dlqr(sys.A, sys.B, sys.Q, sys.H, sys.R)
LQRUtils.dlqr(systype::T) where T<:ModelType = dlqr(PlantModel(systype))
LQRUtils.dlyap(sys::PlantModel) = dlyap(sys.A, sys.Q)
LQRUtils.dlyap(systype::T) where T<:ModelType = dlyap(PlantModel(systype))

struct ControlSystem
	plant::PlantModel  # plant model
	K::Matrix{Float64} # LQR optimal feedback gain
	P::Matrix{Float64} # LQR cost coefficient matrix
end
ControlSystem(plant::PlantModel) = ControlSystem(plant, dlqrp(plant)...) # calculate LQR controller for plant

end # module PlantModels
