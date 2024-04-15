"Model of an inverted pendulum (angle θ) on an acceleration-driven cart (position p) with state x=[p; ṗ; θ; θ̇] and input u=p̈"
struct InvertedPendulum <: ModelType
	Ts::Float64	# sampling period [s]
	m::Float64	# mass [kg]
	d::Float64	# distance of center of mass from pivot [m]
	I::Float64	# moment of inertia around pivot axis [kg⋅m]
	C::Matrix{Float64} # output matrix
    Q::Matrix{Float64} # LQ state cost coefficients
    H::Matrix{Float64} # LQ cross cost coefficients
    R::Matrix{Float64} # LQ input cost coefficient
    W::Matrix{Float64} # state noise covariance matrix
    V::Matrix{Float64} # measurement noise covariance matrix
	InvertedPendulum(Ts, m, d, I; C = Diagonal(ones(4)), Q = Diagonal(Float64[10,10,100,1]), H = zeros(4), R = 1.0, W = Diagonal(zeros(4)), V = Diagonal(zeros(size(C,1)))) =
		new(Ts, m, d, I, asmat(C), asmat(Q), asmat(H), asmat(R), asmat(W), asmat(V))
end

"Model of an inverted pendulum of length `l` on an acceleration-driven cart (thin rod approximation)"
InvertedPendulum(Ts::Float64, l::Float64; kwargs...) =
	InvertedPendulum(Ts, 1.0, l/2.0, l^2/3.0; kwargs...)

# Exact nonlinear model: ẋ = [x₂; u; x₄; g⋅sin(x₃)+K⋅cos(x₃)⋅u]
"Linearized `PlantModel` of and `InvertedPendulum`"
function PlantModel(sys::InvertedPendulum)
	g = 9.8088
	K = sys.m * sys.d / (sys.m * sys.d^2 + sys.I)
	A = [0   1   0   0;
	     0   0   0   0;
	     0   0   0   1;
	     0   0  K*g  0]
	B = asmat([0; 1; 0; K])
	PlantModel(sys.Ts,c2d(A,B,sys.W,sys.Q,sys.H,sys.R,sys.Ts)...,sys.C,sys.V)
end
