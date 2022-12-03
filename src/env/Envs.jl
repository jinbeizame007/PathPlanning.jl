module Envs

using StaticArrays

# from obstacles.jl
export RectObstacle, CircleObstacle, is_inside

# from env.jl
export Env, add_obstacle!, is_inside_any_obstacle

include("obstacles.jl")
include("env.jl")

end # module
