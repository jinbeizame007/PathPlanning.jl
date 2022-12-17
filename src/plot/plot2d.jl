"""
    plot_nodes!(axis::Axis, nodes::Vector{Node{2}})::Scatter

Plot each nodes as small circles.
"""
function plot_nodes!(axis::Axis, nodes::Vector{Node{2}}; color::Symbol=:dodgerblue)::Scatter
    x = [node.position[1] for node in nodes]
    y = [node.position[2] for node in nodes]
    return scatter!(axis, x, y; color=color)
end

"""
    plot_lines!(axis::Axis, nodes::Vector{Node{2}})::LineSegments

Plot all lines on the graph as line segments.
"""
function plot_lines!(axis::Axis, nodes::Vector{Node{2}}; color::Symbol=:dodgerblue)::LineSegments
    x = Vector{Float64}([])
    y = Vector{Float64}([])
    for node in nodes[2:end]
        parent_node = nodes[node.parent]
        push!(x, parent_node.position[1])
        push!(y, parent_node.position[2])
        push!(x, node.position[1])
        push!(y, node.position[2])
    end
    return linesegments!(axis, x, y; color=color)
end

"""
    plot_path!(axis::Axis, nodes::Vector{Node{2}}; color::Symbol=:dodgerblue)::LineSegments

Plot a path that a planner found as line segments.
"""
function plot_path!(axis::Axis, nodes::Vector{Node{2}}; color::Symbol=:dodgerblue)::LineSegments
    x = Vector{Float64}([])
    y = Vector{Float64}([])
    for i in 1:length(nodes)-1
        push!(x, nodes[i].position[1])
        push!(y, nodes[i].position[2])
        push!(x, nodes[i+1].position[1])
        push!(y, nodes[i+1].position[2])
    end
    return linesegments!(axis, x, y; color=color)
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
        resolution::Tuple{Int64,Int64}=(1000,600),
    )::Tuple{Figure, Axis}

Plot nodes, paths, and obstacles that are in the env or the planner.
"""
function plot(
    env::Env,
    planner::AbstractPlanner{2};
    resolution::Tuple{Int64,Int64}=(1000,600),
)::Tuple{Figure, Axis}
    figure = Figure(backgroundcolor = :white; resolution = resolution)
    axis = Axis(figure[1, 1]; limits = (planner.low[1], planner.high[1], planner.low[2], planner.high[2]))

    plot_nodes!(axis, planner.nodes)
    plot_lines!(axis, planner.nodes)
    plot_obstacles!(axis, env.obstacles)
    return figure, axis
end

function animate(
    env::Env,
    planner::AbstractPlanner{2};
    resolution::Tuple{Int64,Int64}=(1000,600),
    framerate::Int64=30,
    file_name::String="path.mp4"
)::Nothing
    figure = Figure(backgroundcolor = :white; resolution = resolution)
    axis = Axis(figure[1, 1]; limits = (planner.low[1], planner.high[1], planner.low[2], planner.high[2]))
    plot_obstacles!(axis, env.obstacles)

    # Plot start and goal node
    scatter!(axis, [planner.start.position[1]], [planner.start.position[2]])
    scatter!(axis, [planner.goal.position[1]], [planner.goal.position[2]])

    num_frames = length(planner.logs)
    frame_indices = 1:num_frames

    plotted_objects = []
    record(figure, file_name, frame_indices; framerate=framerate) do frame_index
        # At frame_index == 1, the movie shows only the start and goal node.
        if frame_index == 1
            return
        end

        # Delete nodes and lines on the previous frame
        while length(plotted_objects) > 0
            delete!(axis, plotted_objects[end])
            pop!(plotted_objects)
        end

        log = planner.logs[frame_index]
        nodes = log["planner"].nodes

        push!(plotted_objects, plot_nodes!(axis, nodes; color=:dodgerblue))
        push!(plotted_objects, plot_lines!(axis, nodes; color=:dodgerblue))
        if log["is_goaled"]
            path = extract_path(log["planner"])
            push!(plotted_objects, plot_nodes!(axis, path; color=:violetred2))
            push!(plotted_objects, plot_path!(axis, path; color=:violetred2))
        end
    end

    return nothing
end
