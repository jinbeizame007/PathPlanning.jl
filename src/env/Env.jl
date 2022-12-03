module Env

using StaticArrays

# from obstacles.jl
export RectObstacle, CircleObstacle, is_inside

include("obstacles.jl")

end # module
