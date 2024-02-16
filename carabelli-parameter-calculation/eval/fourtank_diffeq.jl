## ODE Model

using ModelingToolkit

@parameters t g γ[1:2] A[1:4] a[1:4]
@variables x[1:4](t) u[1:2](t)
@derivatives D'~t

# TODO: add parameters for reference values for linearization?
eqs = [
  D(x[1]) ~ -a[1]/A[1]*sqrt(2*g*x[1]) + a[3]/A[3]*sqrt(2*g*x[3]) + γ[1]/A[1]*u[1],
  D(x[2]) ~ -a[2]/A[2]*sqrt(2*g*x[2]) + a[4]/A[4]*sqrt(2*g*x[4]) + γ[2]/A[2]*u[2],
  D(x[3]) ~ -a[3]/A[3]*sqrt(2*g*x[3]) + (1-γ[2])/A[3]*u[2],
  D(x[4]) ~ -a[4]/A[4]*sqrt(2*g*x[4]) + (1-γ[1])/A[4]*u[1],
  D(u[1]) ~ 0, D(u[2]) ~ 0
]
de = ODESystem(eqs)
f = ODEFunction(de, vcat(x,u), vcat([g], γ, A, a))
f_jac = eval(generate_jacobian(de, vcat(x,u), vcat([g], γ, A, a)))


## Parameter Values

# p = [g, γ₁, γ₂, A₁, A₂, A₃, A₄, a₁, a₂, a₃, a₄]
p = [
  981, # g [cm/s²]
  0.4, # γ₁
  0.4, # γ₂
  50.27, # A₁ [cm²]
  50.27, # A₂ [cm²]
  28.27, # A₃ [cm²]
  28.27, # A₄ [cm²]
  0.233, # a₁ [cm²]
  0.242, # a₂ [cm²]
  0.127, # a₃ [cm²]
  0.127  # a₄ [cm²]
]


## Steady State

using LinearAlgebra

# linearizing transformation: vᵢ = sqrt(xᵢ)

function steady(xc::Dict{Int,Float64}, uc::Dict{Int,Float64}, p)
  @assert issubset(keys(xc), 1:4)
  @assert issubset(keys(uc), 1:2)
  @assert length(xc) + length(uc) == 2
  g, γ₁, γ₂, A₁, A₂, A₃, A₄, a₁, a₂, a₃, a₄ = p

  # [dv₁, dv₂, dv₃, dv₄] = Av * [v₁, v₂, v₃, v₄] + Bv * [u₁, u₂]
  Av = [
    -a₁/A₁    0    a₃/A₃    0   ;
       0   -a₂/A₂    0    a₄/A₄ ;
       0      0   -a₃/A₃    0   ;
       0      0      0   -a₄/A₄
  ] .* sqrt(2g)
  Bv = [
        γ₁/A₁      0   ;
          0      γ₂/A₂ ;
          0  (1-γ₂)/A₃ ;
    (1-γ₁)/A₄      0
  ]

  # As*vs = Bs
  eye = Matrix{Float64}(I,6,6)
  As = vcat( hcat(Av,Bv), eye[collect(keys(xc)),:], eye[collect(keys(uc) .+ 4),:] )
  Bs = vcat( zeros(4), sqrt.(values(xc)), collect(values(uc)) )
  vs = As\Bs
  xs = vs[1:4].^2
  us = vs[5:6]

  (xs, us)
end

xc = Dict{Int,Float64}(1 => 14, 2 => 14) # x values to constrain [cm] (1:4)
uc = Dict{Int,Float64}() # u values to constrain [ml/s] (1:2)
xs, us = steady(xc, uc, p)

J_lin = f_jac(vcat(xs,us),p,0.0)
A_lin = J_lin[1:4,1:4]
B_lin = J_lin[1:4,5:6]

## ODE Problem

using DifferentialEquations

tspan = (0.0, 100.0)
prob = ODEProblem(f, vcat(xs,us), tspan, p)

sol = solve(prob)
