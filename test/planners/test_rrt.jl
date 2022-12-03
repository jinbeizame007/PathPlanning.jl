using PathPlanning.Planners
using StaticArrays

@testset "RRT" begin
    @testset "Node" begin
        position = SVector{2,Float64}(1.0, 2.0)
        node = Node(position)
        @test node.position == position

        node1 = Node(SA[0.0, 0.0])
        node2 = Node(SA[1.0, 1.0])
        @test calc_distance(node1, node2) ≈ 2.0^0.5
        @test calc_distance(node2, node1) ≈ 2.0^0.5
    end

    function get_default_rrt()
        low = SA[0.0, 0.0]
        high = SA[5.0, 5.0]
        start = SA[1.0, 1.0]
        goal = SA[4.0, 4.0]
        return RRT(start, goal, low, high)
    end

    @testset "RRT-Init" begin
        low = SA[0.0, 0.0]
        high = SA[5.0, 5.0]

        start = SA[1.0, 1.0]
        goal = SA[4.0, 4.0]

        goal_sample_rate = 0.1
        step_size = 0.5
        max_iter = 100

        rrt = RRT(start, goal, low, high; goal_sample_rate=goal_sample_rate, step_size=step_size, max_iter=max_iter)
        @test all(rrt.start.position .== start)
        @test all(rrt.goal.position .== goal)
        @test all(rrt.low .== low)
        @test all(rrt.high .== high)
        @test rrt.goal_sample_rate == goal_sample_rate
        @test rrt.step_size == step_size
        @test rrt.max_iter == max_iter

        @test_throws DomainError RRT(start, goal, SA[5.0, 5.0], SA[0.0, 0.0])
        @test_throws DomainError RRT(SA[-1.0, -1.0], goal, low, high)
        @test_throws DomainError RRT(start, SA[-1.0, -1.0], low, high)
    end

    @testset "RRT-Sample" begin
        rrt = get_default_rrt()
        @test all([all(rrt.low .<= sample(rrt).position .<= rrt.high) for _ in 1:100])
    end

    @testset "RRT-GetNearestNodeIndex" begin
        rrt = get_default_rrt()
        nodes = [
            Node(SA[0.0, 0.0]),
            Node(SA[0.0, 5.0]),
            Node(SA[5.0, 0.0]),
            Node(SA[5.0, 5.0]),
        ]
        node = Node(SA[1.0, 1.0])
        rrt.nodes = nodes

        nearest_node_index = get_nearest_node_index(rrt, node)
        nearest_node = rrt.nodes[nearest_node_index]

        @test nearest_node_index == 1
        @test nearest_node == nodes[1]
    end

    @testset "RRT-GetExtendedNode" begin
        rrt = get_default_rrt()
        rrt.nodes = [rrt.start]

        new_node = Node(SA[2.0, 2.0])
        nearest_node_index = get_nearest_node_index(rrt, new_node)
        nearest_node = rrt.nodes[nearest_node_index]

        extended_node = get_extended_node(rrt, nearest_node, new_node)
        
        @test calc_distance(nearest_node, extended_node) ≈ rrt.step_size
        println(extended_node)
        println(nearest_node)
    end

    @testset "RRT-IsNearTheGoal" begin
        rrt = get_default_rrt()
        node_nearest_goal = get_extended_node(rrt, rrt.start, rrt.goal)

        tol = 1e-8
        nearest_node1 = Node(rrt.goal.position + SA[rrt.step_size - tol, 0.0])
        nearest_node2 = Node(rrt.goal.position + SA[0.0, rrt.step_size - tol])
        nearest_node3 = Node(rrt.goal.position - SA[rrt.step_size - tol, 0.0])
        nearest_node4 = Node(rrt.goal.position - SA[0.0, rrt.step_size - tol])

        @test is_near_the_goal(rrt, nearest_node1)
        @test is_near_the_goal(rrt, nearest_node2)
        @test is_near_the_goal(rrt, nearest_node3)
        @test is_near_the_goal(rrt, nearest_node4)
    end

    @testset "RRT-Plan" begin
        low = SA[0.0, 0.0]
        high = SA[5.0, 5.0]
        start = SA[1.0, 1.0]
        goal = SA[4.0, 4.0]

        rrt = RRT(start, goal, low, high; step_size=0.3, max_iter=100)
        path = plan(rrt)
        @test length(path) > 0

        @test all(path[1] == rrt.start)
        @test all(path[end] == rrt.goal)

        tol = 1e-8
        @test all([calc_distance(path[i], path[i+1]) <= rrt.step_size + tol for i in 1:length(path)-1])
    end
end
