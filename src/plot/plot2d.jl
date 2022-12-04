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

function plot_obstacle!(
    axis::Axis,
    obstacle::RectObstacle{2},
)::Nothing
    corner = obstacle.center - obstacle.size./2
    size = obstacle.size
    poly!(Rect(corner[1], corner[2], size[1], size[2]))
    return nothing
end

function plot_obstacle!(
    axis::Axis,
    obstacle::CircleObstacle{2},
)::Nothing
    poly!(Circle(Point2f(obstacle.center[1], obstacle.center[2]), obstacle.radius))
    return nothing
end

function plot_obstacles!(
    axis::Axis,
    obstacles::Vector{AbstractObstacle},
)::Nothing
    for obstacle in obstacles
        plot_obstacle!(axis, obstacle)
    end
    return nothing
end

function plot(
    env::Env,
    planner::AbstractPlanner{2};
    title::Union{String,Nothing}=nothing,
    xlabel::Union{String,Nothing}=nothing,
    ylabel::Union{String,Nothing}=nothing,
    resolution::Tuple{Int64,Int64}=(800,800),
)::Tuple{Figure, Axis}
    figure = Figure(backgroundcolor = :white; resolution = resolution)
    axis = Axis(figure[1, 1])

    plot_nodes!(axis, planner.nodes)
    plot_paths!(axis, planner.nodes)
    plot_obstacles!(axis, env.obstacles)
    return figure, axis
end
