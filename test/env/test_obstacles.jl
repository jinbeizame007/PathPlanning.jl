using PathPlanning.Env
using StaticArrays

@testset "Obstacle" begin
    @testset "RectObstacle" begin
        center = SA[2.0, 2.0]
        size = SA[2.0, 2.0]
        rect_obstacle = RectObstacle(center, size)

        @test is_inside(rect_obstacle, SA[1.0, 1.0])
        @test is_inside(rect_obstacle, SA[1.0, 3.0])
        @test is_inside(rect_obstacle, SA[3.0, 1.0])
        @test is_inside(rect_obstacle, SA[3.0, 3.0])

        @test !is_inside(rect_obstacle, SA[1.0, 0.9])
        @test !is_inside(rect_obstacle, SA[3.1, 3.0])
    end

    @testset "CircleObstacle" begin
        center = SA[2.0, 2.0]
        radius = 1.0
        circle_obstacle = CircleObstacle(center, radius)

        @test is_inside(circle_obstacle, SA[2.0, 1.0])
        @test is_inside(circle_obstacle, SA[1.0, 2.0])
        @test is_inside(circle_obstacle, SA[2.0, 3.0])
        @test is_inside(circle_obstacle, SA[3.0, 2.0])

        @test !is_inside(circle_obstacle, SA[2.0, 0.9])
        @test !is_inside(circle_obstacle, SA[3.1, 2.0])
    end
end
