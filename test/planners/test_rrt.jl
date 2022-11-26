using PathPlanning.Planners
using StaticArrays

@testset "Node" begin
    position = SVector{2,Float64}(1.0, 2.0)
    node = Node(position)
    @test node.position == position

    node1 = Node(SA[0.0, 0.0])
    node2 = Node(SA[1.0, 1.0])
    @test calc_distance(node1, node2) ≈ 2.0^0.5
    @test calc_distance(node2, node1) ≈ 2.0^0.5
end

@testset "RRT-Init" begin
    low = SA[0.0, 0.0]
    high = SA[5.0, 5.0]

    start = SA[1.0, 1.0]
    goal = SA[4.0, 4.0]

    @test_nowarn RRT(start, goal, low, high)

    @test_throws DomainError RRT(start, goal, SA[5.0, 5.0], SA[0.0, 0.0])
    @test_throws DomainError RRT(SA[-1.0, -1.0], goal, low, high)
    @test_throws DomainError RRT(start, SA[-1.0, -1.0], low, high)
end
