module Envs

using StaticArrays

# from obstacles.jl
export AbstractObstacle, RectObstacle, CircleObstacle, is_inside, calc_distance

# from env.jl
export Env, add_obstacle!, is_inside_any_obstacle, create_example_2D_env

include("obstacles.jl")
include("env.jl")

end # module
