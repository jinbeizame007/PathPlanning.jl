module Plot

using PathPlanning.Planners
using PathPlanning.Envs
using StaticArrays
using GLMakie

# from plot2d.jl
export plot, plot_nodes!, plot_paths!

include("plot2d.jl")

end # module
