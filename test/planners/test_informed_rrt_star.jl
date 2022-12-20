using Test
using PathPlanning.Planners
using PathPlanning.Envs
using StaticArrays

@testset "InformedRRTStar" begin
    @testset "InformedRRTStar-Plan" begin
        low = SA[0.0, 0.0]
        high = SA[5.0, 5.0]
        start = SA[1.0, 1.0]
        goal = SA[4.0, 4.0]

        rrt_star = InformedRRTStar(start, goal, low, high; step_size=0.3, max_iter=100)
        path = plan(rrt_star)
        @test length(path) > 0

        @test all(path[1] == rrt_star.start)
        @test all(path[end] == rrt_star.goal)

        tol = 1e-8
        @test all([calc_distance(path[i], path[i+1]) <= rrt_star.step_size + tol for i in 1:length(path)-1])
    end

    @testset "InformedRRTStar-With-Env" begin
        low = SA[0.0, 0.0]
        high = SA[50.0, 30.0]
        start = SA[13.0, 10.0]
        goal = SA[40.0, 20.0]

        env = create_example_2D_env()
        function is_approved(position::SVector{2,Float64})
            return !is_inside_any_obstacle(env, position)
        end

        rrt_star = InformedRRTStar(start, goal, low, high; goal_sample_rate=0.1, step_size=2.0, max_iter=2000, is_approved=is_approved, enable_logging=true)
        path = plan(rrt_star)
        @test length(path) > 0
        @test all(path[1].position .== start)
        @test all(path[end].position .== goal)
        @test all([!is_inside_any_obstacle(env, node.position) for node in path])
    end
end
