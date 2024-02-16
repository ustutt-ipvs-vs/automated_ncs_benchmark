## using 2×4 batch reactor model

# n: state dimension, m: sensor dimension, g: number of replica, h: actuator dimension,
# T: sampling period, δn: max. network delay, p: loss probability,
# θd: sporadic node failure probability, θc: crash failure probability,
# τ: mean processing latency, R: mean time to recover from crash failure
#                            n,       m, g,       h,  T,     δn,    p,   θd,   θc,    τ,   R, sys)
# baseconf = SimConfig(plant.n, plant.p, 3, plant.m, Ts, 0.5e-3, 1e-2, 1e-3, 1e-4, 8e-3,  Ts, sys)

# p ∈ [0.01, 0.02, 0.03, 0.04, 0.05, 0.1, 0.15, 0.2]
# seed = 10
# tsim = 100
# shareOV = true
# hold = false

## cost J:  println(last.(sort( [p => mean(r.cost) for (p,r) in pairs(results)] )))
## unavailability u:  println(last.(sort( [p => r.unavailability for (p,r) in pairs(results)] )))

## Quarts
u = [0.0,       0.0,       0.0,       0.0002,    0.0,       0.0902,    0.8427,      0.9577]
J = [0.0671658, 0.0671099, 0.0670248, 0.0671327, 0.0670148, 0.0801738, 2.90574e129, 1.2363e158]

## Replication
u = [0.0,       0.0,       0.0,       0.0,       0.0,       0.0001,    0.0003,      0.0023]
J = [0.0672082, 0.0675449, 0.0675246, 0.0676147, 0.0677519, 0.0687857, 0.069942,    0.069997] # default estimate sorting
J = [0.067208,  0.0675314, 0.0675564, 0.0675936, 0.0677217, 0.068582,  0.069354,    0.0693227] # state-consistent optimization
J = [0.0672072, 0.0675371, 0.0675564, 0.0675878, 0.0677213, 0.0685715, 0.0692478,   0.0690705] # full optimization
