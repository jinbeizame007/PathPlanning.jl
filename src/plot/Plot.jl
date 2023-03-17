module Plot

using PathPlanning.Planners
using PathPlanning.Envs
using StaticArrays
using GLMakie

# from plot2d.jl
export plot, plot_nodes!, plot_lines!, plot_path!, plot_obstacle!, plot_obstacles!, animate

include("plot2d_rrt.jl")
include("plot2d_stomp.jl")

end # module
