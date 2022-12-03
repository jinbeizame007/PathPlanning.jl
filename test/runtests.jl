using PathPlanning
using Test

@testset "PathPlanning.jl" begin
    include("planners/runtests.jl")
    include("env/runtests.jl")
end
