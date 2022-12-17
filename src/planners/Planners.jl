module Planners

using StaticArrays

# from rrt.jl
export Node, calc_distance, AbstractPlanner, RRT, sample
export get_nearest_node_index, get_extended_node
export is_near_the_goal, extract_path, plan

# from rrt_star.jl
export RRTStar, plan

include("rrt.jl")
include("rrt_star.jl")

end # module
