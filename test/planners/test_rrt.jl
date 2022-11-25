using PathPlanning.Planners
using StaticArrays

@testset "Node" begin
    position = SVector{2,Float64}(1.0, 2.0)
    node = Node(position)
    @test node.position == position
end
