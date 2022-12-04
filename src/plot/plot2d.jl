"""
    plot_nodes!(axis::Axis, nodes::Vector{Node{2}})::Nothing

Plot each nodes as small circles.
"""
function plot_nodes!(axis::Axis, nodes::Vector{Node{2}})::Nothing
    x = [node.position[1] for node in nodes]
    y = [node.position[2] for node in nodes]
    scatter!(axis, x, y)
    return nothing
end

"""
    plot_paths!(axis::Axis, nodes::Vector{Node{2}})::Nothing

Plot all paths as line segments.
If you input all nodes that the planner has, you can plot all paths that the planner explorated,
If you input nodes in a path that the planner found, you can plot a path that the planner found.
"""
function plot_paths!(axis::Axis, nodes::Vector{Node{2}})::Nothing
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

"""
    plot_obstacle!(axis::Axis, obstacle::RectObstacle{2})::Nothing

Plot a RectObstacle.
"""
function plot_obstacle!(axis::Axis, obstacle::RectObstacle{2})::Nothing
    corner = obstacle.center - obstacle.size./2
    size = obstacle.size
    poly!(Rect(corner[1], corner[2], size[1], size[2]))
    return nothing
end

"""
    plot_obstacle!(axis::Axis, obstacle::CircleObstacle{2})::Nothing

Plot a CircleObstacle.
"""
function plot_obstacle!(axis::Axis, obstacle::CircleObstacle{2})::Nothing
    poly!(Circle(Point2f(obstacle.center[1], obstacle.center[2]), obstacle.radius))
    return nothing
end

"""
    plot_obstacles!(axis::Axis, obstacles::Vector{AbstractObstacle})::Nothing

Plot obstacles that can include RectObstacles, CircleObstacles, or both.
"""
function plot_obstacles!(axis::Axis, obstacles::Vector{AbstractObstacle})::Nothing
    for obstacle in obstacles
        plot_obstacle!(axis, obstacle)
    end
    return nothing
end

"""
    plot(
        env::Env,
        planner::AbstractPlanner{2};
        resolution::Tuple{Int64,Int64}=(800,800),
    )::Tuple{Figure, Axis}

Plot nodes, paths, and obstacles that are in the env or the planner.
"""
function plot(
    env::Env,
    planner::AbstractPlanner{2};
    resolution::Tuple{Int64,Int64}=(800,800),
)::Tuple{Figure, Axis}
    figure = Figure(backgroundcolor = :white; resolution = resolution)
    axis = Axis(figure[1, 1])

    plot_nodes!(axis, planner.nodes)
    plot_paths!(axis, planner.nodes)
    plot_obstacles!(axis, env.obstacles)
    return figure, axis
end

"""
    animate(
        env::Env,
        planner::AbstractPlanner{2};
        resolution::Tuple{Int64,Int64}=(800,800),
        framerate::Int64=30,
        file_name::String="path.mp4"
    )::Nothing

Create a movie that shows the planning process.
"""
function animate(
    env::Env,
    planner::AbstractPlanner{2};
    resolution::Tuple{Int64,Int64}=(800,800),
    framerate::Int64=30,
    file_name::String="path.mp4"
)::Nothing
    figure = Figure(backgroundcolor = :white; resolution = resolution)
    axis = Axis(figure[1, 1])
    plot_obstacles!(axis, env.obstacles)

    # Plot start and goal node
    scatter!(axis, [planner.start.position[1]], [planner.start.position[2]])
    scatter!(axis, [planner.goal.position[1]], [planner.goal.position[2]])

    num_frames = length(planner.nodes)
    frame_indices = 2:num_frames

    record(figure, file_name, frame_indices; framerate=framerate) do frame_index
        node = planner.nodes[frame_index]
        parent_node = planner.nodes[node.parent]
        scatter!(axis, [node.position[1]], [node.position[2]]; color=:dodgerblue)
        linesegments!(
            axis,
            [parent_node.position[1], node.position[1]],
            [parent_node.position[2], node.position[2]];
            color=:dodgerblue
        )
    end

    return nothing
end
