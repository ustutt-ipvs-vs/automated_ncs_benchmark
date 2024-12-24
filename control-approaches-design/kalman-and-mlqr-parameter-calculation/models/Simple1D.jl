"Simple 1D system with characteristic polynomial ``λ + s``"
struct Simple1D <: ModelType
	Ts::Float64
	λ::Float64
    Q::Float64
    R::Float64
    W::Float64
    V::Float64
    discrete::Bool
    Simple1D(Ts::Float64, λ::Float64; Q::Float64=1.0, R::Float64=1.0, W::Float64=0.0, V::Float64=0.0, discrete::Bool=false) = new(Ts,λ,Q,R,discrete)
end

"`PlantModel` of a `Simple1D` system in controllable canonical form"
function PlantModel(sys::Simple1D)
	A = reshape([sys.λ],1,1)
	B = reshape([1.0],1,1)
	W = reshape([sys.W],1,1)
	Q = reshape([sys.Q],1,1)
	H = zeros(1,1)
	R = reshape([sys.R],1,1)
    C = reshape([1.0],1,1)
    V = reshape([sys.V],1,1)
    if sys.discrete
        PlantModel(sys.Ts,A,B,W,Q,H,R,C,V)
    else
        PlantModel(sys.Ts,c2d(A,B,W,Q,H,R,sys.Ts)...,C,V)
    end
end
