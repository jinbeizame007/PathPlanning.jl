module Planners

using StaticArrays

# from rrt.jl
export Node, calc_distance, RRT, sample
export get_nearest_node_index, get_extended_node
export plan

include("rrt.jl")

end # module
