using Test

@testset "Planners" begin
    include("test_rrt.jl")
    include("test_rrt_connect.jl")
    include("test_rrt_star.jl")
end
