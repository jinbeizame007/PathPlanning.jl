using Test
using PathPlanning.Planners
using PathPlanning.Envs
using StaticArrays

@testset "RRTStar" begin
    @testset "RRTStar-Plan" begin
        low = SA[0.0, 0.0]
        high = SA[5.0, 5.0]
        start = SA[1.0, 1.0]
        goal = SA[4.0, 4.0]

        rrt = RRTStar(start, goal, low, high; step_size=0.3, max_iter=100)
        path = plan(rrt)
        @test length(path) > 0

        @test all(path[1] == rrt.start)
        @test all(path[end] == rrt.goal)

        tol = 1e-8
        @test all([calc_distance(path[i], path[i+1]) <= rrt.step_size + tol for i in 1:length(path)-1])
    end

    @testset "RRTStar-With-Env" begin
        low = SA[0.0, 0.0]
        high = SA[50.0, 30.0]
        start = SA[1.0, 1.0]
        goal = SA[48.0, 25.0]

        env = create_example_2D_env()
        function is_approved(position::SVector{2,Float64})
            return !is_inside_any_obstacle(env, position)
        end

        rrt = RRTStar(start, goal, low, high; goal_sample_rate=0.05, step_size=1.0, max_iter=1000, is_approved=is_approved)
        path = plan(rrt)
        @test length(path) > 0
        @test all(path[1].position .== start)
        @test all(path[end].position .== goal)
        @test all([!is_inside_any_obstacle(env, node.position) for node in path])
    end
end
