module Envs

using StaticArrays

# from obstacles.jl
export AbstractObstacle, RectObstacle, CircleObstacle, is_inside

# from env.jl
export Env, add_obstacle!, is_inside_any_obstacle, create_example_2D_env, create_example_2D_env_stomp

include("obstacles.jl")
include("env.jl")

end # module
