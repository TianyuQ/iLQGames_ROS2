using Test
using BenchmarkTools

using iLQGames:
    iLQGames,
    iLQSolver,
    GeneralGame,
    Unicycle4D,
    NPlayerUnicycleCost,
    AffineStrategy,
    SystemTrajectory,
    dynamics,
    n_states,
    n_controls,
    horizon,
    player_costs,
    quadraticize,
    _quadraticize_ad,
    generate_nplayer_navigation_game,
    solve!,
    ProximityCost,
    solve

using iLQGames.TestUtils
using StaticArrays
using LinearAlgebra

# generate a game
T_horizon = 15.
ΔT = 0.1

"--------------------------------- Unicycle4D ---------------------------------"

# global x01 = @SVector [-3., 0., 0., 0.]
# global x02 = @SVector [0.,  3., 0, 0.]
# global x03 = @SVector [-3.,  3., 0, 0.]
# global x0 = vcat(x01, x02, x03)
# goal states (goal position of other player with opposite orientation)
xg1 = @SVector [3., 0., 0., 0.]
xg2 = @SVector [0., -3., 0, 0.]
xg3 = @SVector [3., -3., 0, 0.]
g = generate_nplayer_navigation_game(Unicycle4D, NPlayerUnicycleCost, T_horizon,
                                     ΔT, xg1, xg2, xg3;
                                     proximitycost=ProximityCost([2.0, 2.0, 2.0],
                                                                 [50.0, 50.0, 50.0]))
dyn = dynamics(g)
nx = n_states(dyn)
nu = n_controls(dyn)
pcs = player_costs(g)
h = horizon(g)
zero_op = zero(SystemTrajectory{h, ΔT, nx, nu})

# quad_sanity_check(g)

# solve the lq game
solver = iLQSolver(g; state_regularization=5.0, control_regularization=5.0)

# function game_solving(x0=x0)
    # converged, trajectory, strategies = solve(g, solver, x0)
    # println("trajectory: ", trajectory.x[end])
    # println("strategy: ", trajectory.u[end])
#     return trajectory.x[end]
# end

function nplayer_navigation(x0)
    x01 = SVector{4, Float64}(x0[1:4])
    x02 = SVector{4, Float64}(x0[5:8])
    x03 = SVector{4, Float64}(x0[9:12])

    x0 = vcat(x01, x02, x03)

    converged, trajectory, strategies = solve(g, solver, x0)

    return trajectory.x[2], trajectory.u[1]
end