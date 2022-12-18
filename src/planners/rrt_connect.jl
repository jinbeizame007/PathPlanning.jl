"""
    RRTConnect{N}

A planner using RRT-Connect.

# Fields
- `start::Node{N}`: start position in the search space
- `goal::Node{N}`: goal position in the search space
- `low::SVector{N,Float64}`: lower bounds of the search space
- `high::SVector{N,Float64}`: upper bounds of the search space
- `nodes::Vector{Node{N}}`: all nodes
- `nodes_from_start::Vector{Node{N}}`: nodes configure the graph
- `nodes_from_goal::Vector{Node{N}}`: nodes configure the graph
- `step_size::Float64`: maximum distance between each node
- `max_iter::Int64`: maximum the number of the iterations
- `is_approved::Union{Function, Nothing}`: a function that returns if the node can be added the graph
- `enable_logging::Bool`: whether to log (is_goaled, RRTStar instance) for each steps in planning
- `logs::Vector{Dict{String, Any}}`: (is_goaled, RRT instance) for each steps in planning
"""
mutable struct RRTConnect{N} <: AbstractRRT{N}
    start::Node{N}
    goal::Node{N}
    low::SVector{N,Float64}
    high::SVector{N,Float64}
    nodes::Vector{Node{N}}
    nodes_from_start::Vector{Node{N}}
    nodes_from_goal::Vector{Node{N}}
    step_size::Float64
    max_iter::Int64
    is_approved::Union{Function, Nothing}
    enable_logging::Bool
    logs::Vector{Dict{String, Any}}
end

"""
    RRTConnect{N} <: AbstractRRT{N}
        start::Node{N}
        goal::Node{N}
        low::SVector{N,Float64}
        high::SVector{N,Float64}
        nodes::Vector{Node{N}}
        nodes_from_start::Vector{Node{N}}
        nodes_from_goal::Vector{Node{N}}
        step_size::Float64
        max_iter::Int64
        is_approved::Union{Function, Nothing}
        enable_logging::Bool
        logs::Vector{Dict{String, Any}}
    end

Constructor of RRTConnect struct.
The `step_size` is initiated as 1/20th of the distance between the start node and the start node by default.

# Arguments
- `start::Node{N}`: start position in the search space
- `goal::Node{N}`: goal position in the search space
- `low::SVector{N,Float64}`: lower bounds of the search space
- `high::SVector{N,Float64}`: upper bounds of the search space
- `step_size::Float64`: maximum distance between each node
- `max_iter::Int64`: maximum the number of the iterations
- `is_approved::Union{Function, Nothing}`: a function that returns if the node can be added the graph
- `enable_logging::Bool`: whether to log (is_goaled, RRTStar instance) for each steps in planning
"""
function RRTConnect(
    start::SVector{N,Float64},
    goal::SVector{N,Float64},
    low::SVector{N,Float64},
    high::SVector{N,Float64};
    step_size::Union{Float64,Nothing} = nothing,
    max_iter::Int64 = 500,
    is_approved::Union{Function, Nothing} = nothing,
    enable_logging::Bool = false
)::RRTConnect{N} where {N}
    if any(high .<= low)
        throw(DomainError((low, high), "The `low` ($low) should be lower than `high` ($high)."))
    end

    if !all(low .<= start .<= high)
        throw(DomainError(start, "The `start` ($start) should be between `low` ($low) and `high` ($high)."))
    end

    if !all(low .<= goal .<= high)
        throw(DomainError(goal, "The `goal` ($goal) should be between `low` ($low) and `high` ($high)."))
    end

    nodes = Vector{Node{N}}([])
    nodes_from_start = Vector{Node{N}}([])
    nodes_from_goal = Vector{Node{N}}([])
    if isnothing(step_size)
        step_size = sum((goal - start).^2.0)^0.5 / 20.0
    end

    logs = Vector{Dict{String, Any}}([])
    return RRTConnect{N}(Node(start), Node(goal), low, high, nodes, nodes_from_start, nodes_from_goal, step_size, max_iter, is_approved, enable_logging, logs)
end

function reverse_path(path::Vector{Node{N}}, start_node_index::Int64)::Vector{Node{N}} where N
    path = reverse(deepcopy(path))
    parent_index = start_node_index
    for i in 2:length(path)
        path[i].parent = parent_index
        parent_index = path[i-1].parent
    end
    path[1].parent = nothing
    return path
end

"""
    extract_path(nodes::Vector{Node{N}}, end_node_index::Int64) where N

Return the path from the node at `end_node_index` in nodes.
"""
function extract_path(nodes::Vector{Node{N}}, end_node_index::Int64) where N
    path = Vector{Node{N}}([])

    # Extract path
    node = nodes[end_node_index]
    while true
        push!(path, node)

        if isnothing(node.parent)
            break
        end
        node = nodes[node.parent]
    end
    path = reverse(path)
end

"""
    get_all_nodes(planner::RRTConnect{N})::Vector{Node{N}} where {N}

Combine and return nodes from start and nodes from goal.
"""
function get_all_nodes(planner::RRTConnect{N})::Vector{Node{N}} where {N}
    diff_index = length(planner.nodes_from_start)
    all_nodes = vcat(planner.nodes_from_start, deepcopy(planner.nodes_from_goal))
    for node in all_nodes[length(planner.nodes_from_start)+1:end]
        if !isnothing(node.parent)
            node.parent += diff_index
        end
    end
    return all_nodes
end

"""
    plan(planner::RRTConnect{N})::Vector{Node{N}} where {N}

Find and return a path from the start node `planner.start` to the goal node `planner.goal` with RRT-Connect algorithm.

# Arguments
- planner::RRTConnect{N}: a planner using RRT-Connect

# Returns
- `path::Vector{Node{N}}`: path (sequence of nodes) from the start node `planner.start` to the goal node `planner.goal`
"""
function plan(planner::RRTConnect{N})::Vector{Node{N}} where {N}
    planner.nodes_from_start = [planner.start]
    planner.nodes_from_goal = [planner.goal]

    is_goaled = false
    logs = Vector{Dict{String, Any}}([])
    if planner.enable_logging
        planner_copy = deepcopy(planner)
        planner_copy.nodes = get_all_nodes(planner_copy)
        log = Dict("is_goaled" => is_goaled, "planner" => planner_copy)
        push!(logs, log)
    end

    for i in 1:planner.max_iter
        # Sample a node
        new_node = sample(planner)

        # Select tree with fewer nodes as the tree that will be added the new node
        is_sampled_for_start_tree = length(planner.nodes_from_start) <= length(planner.nodes_from_goal)
        sample_target_nodes = is_sampled_for_start_tree ? planner.nodes_from_start : planner.nodes_from_goal
        extend_target_nodes = is_sampled_for_start_tree ? planner.nodes_from_goal : planner.nodes_from_start

        # Get the nearest node
        nearest_node_index = get_nearest_node_index(sample_target_nodes, new_node)
        nearest_node = sample_target_nodes[nearest_node_index]

        # Get the extended node
        distance_from_nearest_node = calc_distance(nearest_node, new_node)
        if planner.step_size < distance_from_nearest_node
            new_node = get_extended_node(planner, nearest_node, new_node)
        end

        # Check if the node is approved (e.g., if the node is inside any obstacle.)
        if !isnothing(planner.is_approved) && !planner.is_approved(new_node.position)
            continue
        end

        # Add the new node to the graph
        new_node.parent = nearest_node_index
        push!(sample_target_nodes, new_node)
        new_node_index = length(sample_target_nodes)

        path = Vector{Node{N}}([])
        while true
            # Get the nearest node in extended_target_nodes
            nearest_node_index = get_nearest_node_index(extend_target_nodes, new_node)
            nearest_node = extend_target_nodes[nearest_node_index]

            is_goaled = calc_distance(new_node, nearest_node) <= planner.step_size
            if is_goaled
                # Combine path from start and path from goal
                end_node_index_from_start = is_sampled_for_start_tree ? new_node_index : nearest_node_index
                end_node_index_from_goal = is_sampled_for_start_tree ? nearest_node_index : new_node_index
                
                path_from_start = extract_path(planner.nodes_from_start, end_node_index_from_start)
                path_from_goal = extract_path(planner.nodes_from_goal, end_node_index_from_goal)
                path_to_goal = reverse_path(path_from_goal, end_node_index_from_goal)
                path_to_goal[1].parent = end_node_index_from_start
                path = vcat(path_from_start, path_to_goal)
                break
            else
                extended_node = get_extended_node(planner, new_node, nearest_node)
                if !isnothing(planner.is_approved) && !planner.is_approved(extended_node.position)
                    break
                end

                if planner.enable_logging
                    planner_copy = deepcopy(planner)
                    planner_copy.nodes = get_all_nodes(planner_copy)
                    log = Dict("is_goaled" => is_goaled, "planner" => planner_copy)
                    push!(logs, log)
                end

                # Add the new node to the graph
                extended_node.parent = new_node_index
                push!(sample_target_nodes, extended_node)
                new_node = extended_node
                new_node_index = length(sample_target_nodes)
            end
        end

        if planner.enable_logging
            planner_copy = deepcopy(planner)
            planner_copy.nodes = get_all_nodes(planner_copy)
            log = Dict("is_goaled" => is_goaled, "planner" => planner_copy)
            if is_goaled
                log["path"] = path
            end
            push!(logs, log)
        end

        if is_goaled
            planner.logs = logs
            return path
        end
    end

    planner.logs = logs
    return Vector{Node{N}}([])
end
