using Test
using PathPlanning.Envs
using PathPlanning.Planners
using PathPlanning.Plot
using StaticArrays

@testset "STOMP" begin
    @testset "STOMP-Plan" begin
        low = SA[0.0, 0.0]
        high = SA[50.0, 30.0]
        start = SA[1.0, 1.0]
        goal = SA[48.0, 25.0]

        env = create_example_2D_env2()
        function cost_func(stomp::STOMP{2}, paths::Array{Float64})
            costs = zeros(stomp.path_length, stomp.num_samples)
            costs .+= calc_distance_cost(stomp, env, paths)
            costs .+= 0.03 * calc_torque_cost(stomp, paths)
            return costs
        end

        stomp = STOMP(start, goal, low, high; cost_func=cost_func, enable_logging=true)
        path = plan(stomp)
        @test length(path) > 1
        # animate(env, stomp)
    end
end
