"Simple 2D system with characteristic polynomial ``a₀ + a₁⋅s + s²``"
struct Simple2D <: ModelType
	Ts::Float64
	a₀::Float64
	a₁::Float64
    Q::Vector{Float64}
    R::Float64
    w::Vector{Float64}
    C::Matrix{Float64}
    v::Vector{Float64}
    discrete::Bool
    Simple2D(Ts::Float64, a₀::Float64, a₁::Float64; Q::Vector{Float64}=Float64[1,1], R::Float64=1.0, w::Vector{Float64}=Float64[0,0],
        C::Matrix{Float64}=Matrix{Float64}(I,2,2), v::Float64=Float64[0,0], discrete::Bool=false) = new(Ts,a₀,a₁,Q,R,w,C,v,discrete)
end

"Simple 2D system with poles at `λ₁` and `λ₂`"
function Simple2DPole(Ts::Float64, λ₁::Number, λ₂::Number; kwargs...)
    @assert isreal(λ₁) ? isreal(λ₂) : λ₁ == conj(λ₂)
    Simple2D(Ts, Float64(λ₁*λ₂), Float64(-λ₁-λ₂); kwargs...)
end

"`PlantModel` of a `Simple2D` system in controllable canonical form"
function PlantModel(sys::Simple2D)
    @assert length(sys.w) == 2
	A = [0.0 1.0; -sys.a₀ -sys.a₁]
	B = [0.0 1.0]'
	W = diagm(0 => sys.w)
	Q = diagm(0 => sys.Q)
	H = zeros(2,1)
	R = reshape([sys.R],1,1)
    C = sys.C
    V = diagm(0 => sys.v)
    if sys.discrete
        PlantModel(sys.Ts,A,B,W,Q,H,R,C,V)
    else
        PlantModel(sys.Ts,c2d(A,B,W,Q,H,R,sys.Ts)...,C,V)
    end
end
