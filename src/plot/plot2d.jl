function plot_nodes!(
    axis::Axis,
    nodes::Vector{Node{2}},
)::Nothing
    x = [node.position[1] for node in nodes]
    y = [node.position[2] for node in nodes]
    scatter!(axis, x, y)
    return nothing
end

function plot_paths!(
    axis::Axis,
    nodes::Vector{Node{2}},
)::Nothing
    x = Vector{Float64}([])
    y = Vector{Float64}([])
    for node in nodes[2:end]
        parent_node = nodes[node.parent]
        push!(x, parent_node.position[1])
        push!(y, parent_node.position[2])
        push!(x, node.position[1])
        push!(y, node.position[2])
    end
    linesegments!(axis, x, y)
    return nothing
end

function plot(
    env::Env,
    planner::AbstractPlanner{2};
    title::Union{String,Nothing}=nothing,
    xlabel::Union{String,Nothing}=nothing,
    ylabel::Union{String,Nothing}=nothing,
    resolution::Tuple{Int64,Int64}=(800,800),
)::Figure
    figure = Figure(backgroundcolor = :white; resolution = resolution)
    axis = Axis(figure[1, 1])

    plot_nodes!(axis, planner.nodes)
    plot_paths!(axis, planner.nodes)
    return figure
end
