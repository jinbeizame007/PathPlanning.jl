abstract type AbstractRRTStar{N} <: AbstractRRT{N} end

"""
    RRTStar{N}

A planner using RRT*.

# Fields
- `start::Node{N}`: start position in the search space
- `goal::Node{N}`: goal position in the search space
- `low::SVector{N,Float64}`: lower bounds of the search space
- `high::SVector{N,Float64}`: upper bounds of the search space
- `nodes::Vector{Node{N}}`: nodes configure tje graph
- `goal_sample_rate::Float64`: rate of sampling the goal node
- `step_size::Float64`: maximum distance between each node
- `max_iter::Int64`: maximum the number of the iterations
- `is_approved::Union{Function, Nothing}`: a function that returns if the node can be added the graph
- `enable_logging::Bool`: whether to log (is_goaled, RRTStar instance) for each steps in planning
- `logs::Vector{Dict{String, Any}}`: (is_goaled, RRTStar instance) for each steps in planning
"""
mutable struct RRTStar{N} <: AbstractRRTStar{N}
    start::Node{N}
    goal::Node{N}
    low::SVector{N,Float64}
    high::SVector{N,Float64}
    nodes::Vector{Node{N}}
    goal_sample_rate::Float64
    step_size::Float64
    max_iter::Int64
    is_approved::Union{Function, Nothing}
    enable_logging::Bool
    logs::Vector{Dict{String, Any}}
end

"""
    RRTStar(
        start::SVector{N,Float64},
        goal::SVector{N,Float64},
        low::SVector{N,Float64},
        high::SVector{N,Float64};
        goal_sample_rate::Float64 = 0.2,
        step_size::Union{Float64,Nothing} = nothing,
        max_iter::Int64 = 500,
        is_approved::Union{Function, Nothing} = nothing,
        enable_logging::Bool = false
    ) where {N}

Constructor of RRTStar struct.
The `step_size` is initiated as 1/20th of the distance between the start node and the start node by default.

# Arguments
- `start::Node{N}`: start position in the search space
- `goal::Node{N}`: goal position in the search space
- `low::SVector{N,Float64}`: lower bounds of the search space
- `high::SVector{N,Float64}`: upper bounds of the search space
- `goal_sample_rate::Float64`: rate of sampling the goal node
- `step_size::Float64`: maximum distance between each node
- `max_iter::Int64`: maximum the number of the iterations
- `is_approved::Union{Function, Nothing}`: a function that returns if the node can be added the graph
- `enable_logging::Bool`: whether to log (is_goaled, RRTStar instance) for each steps in planning
"""
function RRTStar(
    start::SVector{N,Float64},
    goal::SVector{N,Float64},
    low::SVector{N,Float64},
    high::SVector{N,Float64};
    goal_sample_rate::Float64 = 0.2,
    step_size::Union{Float64,Nothing} = nothing,
    max_iter::Int64 = 500,
    is_approved::Union{Function, Nothing} = nothing,
    enable_logging::Bool = false
)::RRTStar{N} where {N}
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
    if isnothing(step_size)
        step_size = sum((goal - start).^2.0)^0.5 / 20.0
    end

    logs = Vector{Dict{String, Any}}([])
    return RRTStar{N}(Node(start), Node(goal), low, high, nodes, goal_sample_rate, step_size, max_iter, is_approved, enable_logging, logs)
end

"""
    add_node!(rrt_star::AbstractRRTStar{N}, new_node::Node{N}, parent_node_index::Int64)::Nothing where {N}

Add new node with the parent index and the cost to the graph.
The cost of the new node is the cost of the parent node and the distance between the parent node and the new node.
"""
function add_node!(rrt_star::AbstractRRTStar{N}, new_node::Node{N}, parent_node_index::Int64)::Nothing where {N}
    parent_node = rrt_star.nodes[parent_node_index]

    # Add new_node to nodes
    new_node.parent = parent_node_index
    new_node.cost = parent_node.cost + calc_distance(parent_node, new_node)
    push!(rrt_star.nodes, new_node)

    # Add new_node_index to children of the parent node
    new_node_index = length(rrt_star.nodes)
    push!(parent_node.children, new_node_index)
    return nothing
end

"""
    get_shortest_parent_node_index(rrt::AbstractRRTStar{N}, new_node::Node{N})::Int64 where {N}

Get the parent node that minimizes the length between the start node and the new node.
"""
function get_shortest_parent_node_index(rrt::AbstractRRTStar{N}, new_node::Node{N})::Int64 where {N}
    tol = 1e-8
    distances = [calc_distance(node, new_node) <= rrt.step_size + tol ? node.cost : 1e10 + calc_distance(node, new_node) for node in rrt.nodes]
    _, index = findmin(distances)
    return index
end

"""
    get_near_node_indices(rrt::AbstractRRT{N}, new_node::Node{N}, distance::Float64)::Vector{Int64} where {N}

Get nodes within `distance` from the `new_node`.
"""
function get_near_node_indices(rrt::AbstractRRT{N}, new_node::Node{N}, distance::Float64)::Vector{Int64} where {N}
    return Vector{Int64}([i for i in 1:length(rrt.nodes) if calc_distance(rrt.nodes[i], new_node) <= distance])
end

"""
    update_costs!(planner::AbstractRRTStar{N}, node::Node{N}, diff_cost::Float64)::Nothing where N

Update costs of all children recursively.
"""
function update_costs!(planner::AbstractRRTStar{N}, node::Node{N}, diff_cost::Float64)::Nothing where N
    node.cost += diff_cost
    for child_node_index in node.children
        child_node = planner.nodes[child_node_index]
        update_costs!(planner, child_node, diff_cost)
    end
    return nothing
end

"""
    rewire_near_nodes!(rrt_star::AbstractRRTStar{N}, near_node_indices::Vector{Int64}, new_node_index::Int64) where {N}

Change parent nodes to the new node if the new cost is smaller than the current cost.
"""
function rewire_near_nodes!(rrt_star::AbstractRRTStar{N}, near_node_indices::Vector{Int64}, new_node_index::Int64) where {N}
    new_node = rrt_star.nodes[new_node_index]
    for near_node_index in near_node_indices
        near_node = rrt_star.nodes[near_node_index]

        new_cost = new_node.cost + calc_distance(new_node, near_node)
        if new_cost < near_node.cost
            # Delete near_node from children of the current parent node of the near node
            pop!(rrt_star.nodes[near_node.parent].children, near_node_index)

            # Set new_node as parent of near_node
            near_node.parent = new_node_index
            # Add near_node as children of new_node
            push!(new_node.children, near_node_index)

            diff_cost = new_cost - near_node.cost
            update_costs!(rrt_star, near_node, diff_cost)
        end
    end

    return nothing
end

"""
    plan(rrt_star::RRTStar{N})::Vector{Node{N}} where {N}

Find and return a path from the start node `rrt_star.start` to the goal node `rrt_star.goal` with RRT* algorithm.

# Arguments
- rrt_star::RRTStar{N}: a planner using RRT*

# Returns
- `path::Vector{Node{N}}`: path (sequence of nodes) from the start node `rrt.start` to the goal node `rrt.goal`
"""
function plan(rrt_star::RRTStar{N})::Vector{Node{N}} where {N}
    rrt_star.nodes = [rrt_star.start]
    is_goaled = false

    logs = Vector{Dict{String, Any}}([])
    if rrt_star.enable_logging
        log = Dict("is_goaled" => is_goaled, "planner" => deepcopy(rrt_star))
        push!(logs, log)
    end

    for i in 1:rrt_star.max_iter
        # Sample a node
        new_node = if rand() < rrt_star.goal_sample_rate
            Node(rrt_star.goal.position)
        else
            sample(rrt_star)
        end

        # Get the nearest node
        nearest_node_index = get_nearest_node_index(rrt_star.nodes, new_node)
        nearest_node = rrt_star.nodes[nearest_node_index]

        # Get a node extended from the nearest node
        distance_from_nearest_node = calc_distance(nearest_node, new_node)
        if rrt_star.step_size < distance_from_nearest_node
            new_node = get_extended_node(rrt_star, nearest_node, new_node)
        end

        # Check if the node is approved (e.g., if the node is inside any obstacle.)
        if !isnothing(rrt_star.is_approved) && !rrt_star.is_approved(new_node.position)
            continue
        end

        # Add the new node to the graph
        parent_node_index = get_shortest_parent_node_index(rrt_star, new_node)
        add_node!(rrt_star, new_node, parent_node_index)
        new_node_index = length(rrt_star.nodes)

        # Add goal node to the graph if the new node is near the goal node
        if !is_goaled && is_near_the_goal(rrt_star, new_node)
            new_node_index = length(rrt_star.nodes)
            add_node!(rrt_star, rrt_star.goal, new_node_index)
            is_goaled = true
        end

        # Rewire near nodes
        near_node_indices = get_near_node_indices(rrt_star, new_node, rrt_star.step_size)
        rewire_near_nodes!(rrt_star, near_node_indices, new_node_index)

        if rrt_star.enable_logging
            log = Dict("is_goaled" => is_goaled, "planner" => deepcopy(rrt_star))
            push!(logs, log)
        end
    end

    rrt_star.logs = logs
    return extract_path(rrt_star)
end
