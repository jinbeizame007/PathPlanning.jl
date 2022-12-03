module Planners

using StaticArrays

# from rrt.jl
export Node, calc_distance, AbstractSampler, RRT, sample
export get_nearest_node_index, get_extended_node
export is_near_the_goal, plan

include("rrt.jl")

end # module
