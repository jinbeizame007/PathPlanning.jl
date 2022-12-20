using PathPlanning.Envs
using StaticArrays

@testset "Env" begin
    obstacles = [
        RectObstacle(SA[2.0, 2.0], SA[2.0, 2.0]),
        CircleObstacle(SA[4.0, 4.0], 1.0),
    ]
    env1 = Env(obstacles)
    env2 = Env()
    add_obstacle!(env2, obstacles[1]) 
    add_obstacle!(env2, obstacles[2])
    
    @test env1.num_obstacles == 2
    @test env2.num_obstacles == 2
    for i in 1:length(obstacles)
        @test env1.obstacles[i] == env2.obstacles[i]
    end

    @test is_inside_any_obstacle(env1, SA[2.0, 2.0])
    @test is_inside_any_obstacle(env1, SA[4.0, 4.0])
    @test !is_inside_any_obstacle(env1, SA[0.0, 2.0])
    @test !is_inside_any_obstacle(env1, SA[6.0, 0.0])
end
