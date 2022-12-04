using Test
using PathPlanning.Planners
using PathPlanning.Envs
using PathPlanning.Plot
using StaticArrays

@testset "Plot2D" begin
    low = SA[0.0, 0.0]
    high = SA[50.0, 30.0]
    start = SA[1.0, 1.0]
    goal = SA[48.0, 25.0]

    env = create_example_2D_env()
    function is_approved(position::SVector{2,Float64})
        return !is_inside_any_obstacle(env, position)
    end

    rrt = RRT(start, goal, low, high; step_size=1.0, max_iter=1000, is_approved=is_approved)
    path = plan(rrt)

    @test_nowarn plot(env, rrt)

    figure, axis = plot(env, rrt)
    @test_nowarn plot_nodes!(axis, rrt.nodes)
    @test_nowarn plot_paths!(axis, rrt.nodes)
    for obstacle in env.obstacles
        @test_nowarn plot_obstacle!(axis, obstacle)
    end
    @test_nowarn plot_obstacles!(axis, env.obstacles)
end
