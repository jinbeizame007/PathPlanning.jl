using Test

@testset "Planners" begin
    include("test_rrt.jl")
    include("test_rrt_connect.jl")
    include("test_rrt_star.jl")
    include("test_informed_rrt_star.jl")
    include("test_stomp.jl")
end
