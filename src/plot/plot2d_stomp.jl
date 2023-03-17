function plot_path!(axis::Axis, plotted_objects, stomp::STOMP{2}, path::Matrix{Float64}; color::Symbol=:dodgerblue)::Nothing
    x = Vector{Float64}([])
    y = Vector{Float64}([])
    for i in 2:size(path,2)
        push!(x, path[1,i-1])
        push!(y, path[2,i-1])
        push!(x, path[1,i])
        push!(y, path[2,i])
    end
    push!(plotted_objects, linesegments!(axis, x, y; color=color))

    x = [path[1,i] for i in 1:size(path,2)]
    y = [path[2,i] for i in 1:size(path,2)]
    push!(plotted_objects, scatter!(axis, x, y; color=color))
    return nothing
end

function animate(
    env::Env,
    stomp::STOMP{2};
    resolution::Tuple{Int64,Int64}=(1000,600),
    framerate::Int64=20,
    file_name::String="path.mp4"
)::Nothing
    figure = Figure(backgroundcolor = :white; resolution = resolution)
    axis = Axis(figure[1, 1]; limits = (stomp.low[1], stomp.high[1], stomp.low[2], stomp.high[2]))
    plot_obstacles!(axis, env.obstacles)

    # Plot start and goal node
    scatter!(axis, [stomp.start[1]], [stomp.start[2]])
    scatter!(axis, [stomp.goal[1]], [stomp.goal[2]])

    num_frames = length(stomp.logs)
    println("num_frames: $num_frames")
    frame_indices = 1:num_frames

    plotted_objects = []
    record(figure, file_name, frame_indices; framerate=framerate) do frame_index
        # Delete nodes and lines on the previous frame
        while length(plotted_objects) > 0
            delete!(axis, plotted_objects[end])
            pop!(plotted_objects)
        end

        log = stomp.logs[frame_index]
        mean = log["mean"]
        sampled_paths = log["sampled_paths"]

        for i in 1:stomp.num_samples
            plot_path!(axis, plotted_objects, stomp, sampled_paths[:,:,i])
        end
        plot_path!(axis, plotted_objects, stomp, mean; color=:violetred2)
    end

    return nothing
end
