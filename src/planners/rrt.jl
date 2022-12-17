"""
    Node{N}

Node configures a graph.

# Fields
- `position::SVector{N, Float64}`: position of the node in the search space
- `parent::Union{Int64, Nothing}`: index of the parent node
"""
mutable struct Node{N}
    position::SVector{N, Float64}
    parent::Union{Int64, Nothing}
    cost::Float64
end

"""
    Node(position::SVector{N, Float64}) where {N}

Constructor of the Node struct.
The index of the parent node is initiated as nothing.

# Arguments

- `position::SVector{N, Float64}`: position of the node in the search space
"""
function Node(position::SVector{N, Float64})::Node{N} where {N}
    return Node{N}(position, nothing, 0.0)
end

function Node(position::SVector{N, Float64}, cost::Float64)::Node{N} where {N}
    return Node{N}(position, nothing, cost)
end

"""
    calc_distance(node1::Node{N}, node2::Node{N})::Float64 where {N}

Calculate the Euclidean distance between the two nodes in the N-dimensional space.
"""
function calc_distance(node1::Node{N}, node2::Node{N})::Float64 where {N}
    return sum((node1.position - node2.position).^2.0)^0.5
end

abstract type AbstractPlanner{N} end
abstract type AbstractRRT{N} <: AbstractPlanner{N} end

"""
    RRT{N}

A planner using RRT.

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
- `logs::Vector{Dict{String, Any}}`: (is_goaled, RRT instance) for each steps in planning
"""
mutable struct RRT{N} <: AbstractRRT{N}
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
    RRT(
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

Constructor of RRT struct.
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
function RRT(
    start::SVector{N,Float64},
    goal::SVector{N,Float64},
    low::SVector{N,Float64},
    high::SVector{N,Float64};
    goal_sample_rate::Float64 = 0.2,
    step_size::Union{Float64,Nothing} = nothing,
    max_iter::Int64 = 500,
    is_approved::Union{Function, Nothing} = nothing,
    enable_logging::Bool = false
)::RRT{N} where {N}
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
    return RRT{N}(Node(start), Node(goal), low, high, nodes, goal_sample_rate, step_size, max_iter, is_approved, enable_logging, logs)
end

"""
    sample(rrt::AbstractRRT{N})::Node{N} where {N}

Sample a node from the search space.
"""
function sample(rrt::AbstractRRT{N})::Node{N} where {N}
    return Node((@SVector rand(N)) .* (rrt.high - rrt.low) - rrt.low)
end

"""
    get_nearest_node_index(rrt::AbstractRRT{N}, new_node::Node{N})::Int64 where {N}

Return the index of the node nearest to the given node in the graph.
"""
function get_nearest_node_index(rrt::AbstractRRT{N}, new_node::Node{N})::Int64 where {N}
    distances = [calc_distance(node, new_node) for node in rrt.nodes]
    _, index = findmin(distances)
    return index
end

"""
    get_extended_node(rrt::RRT{N}, nearest_node::Node{N}, new_node::Node{N})::Node{N} where {N}

Create and return a node at the position from the nearest node to the new node as close as by the step size.

# Arguments
- `rrt::AbstractRRT{N}`: a planner using RRT based algorithm
- `nearest_node::Node{N}`: the node nearest to the new node in the graph
- `new_node::Node{N}`: the node newly sampled
"""
function get_extended_node(rrt::AbstractRRT{N}, nearest_node::Node{N}, new_node::Node{N})::Node{N} where {N}
    delta = new_node.position - nearest_node.position
    distance = calc_distance(nearest_node, new_node)
    
    position = nearest_node.position + delta * (rrt.step_size / distance)
    extended_node = Node(position)
    return extended_node
end

"""
    is_near_the_goal(rrt::AbstractRRT{N}, node::Node{N}) where {N}

Return if given node is near the goal (distance from goal is smaller than the `rrt.step_size`.)

# Arguments
- `rrt::AbstractRRT{N}`: a planner using RRT based algorithm
- `node::Node{N}`: a node in the search space
"""
function is_near_the_goal(rrt::AbstractRRT{N}, node::Node{N}) where {N}
    distance_from_goal = calc_distance(rrt.goal, node)
    return distance_from_goal <= rrt.step_size
end

"""
    extract_path(rrt::AbstractRRT{N}) where N

Return a path from start node to goal node by extracting nodes.

# Arguments
- rrt::AbstractRRT{N}: a planner using RRT based algorithm
"""
function extract_path(rrt::AbstractRRT{N}) where N
    path = Vector{Node{N}}([])

    node = rrt.goal
    while true
        push!(path, node)

        if isnothing(node.parent)
            break
        end
        node = rrt.nodes[node.parent]
    end

    return reverse(path)
end

"""
    plan(rrt::RRT{N})::Vector{Node{N}} where {N}

Find and return a path from the start node `rrt.start` to the goal node `rrt.goal` with RRT algorithm.

# Arguments
- rrt::RRT{N}: a planner using RRT

# Returns
- `path::Vector{Node{N}}`: path (sequence of nodes) from the start node `rrt.start` to the goal node `rrt.goal`
"""
function plan(rrt::RRT{N})::Vector{Node{N}} where {N}
    rrt.nodes = [rrt.start]

    is_goaled = false
    logs = Vector{Dict{String, Any}}([])
    if rrt.enable_logging
        log = Dict("is_goaled" => is_goaled, "planner" => deepcopy(rrt))
        push!(logs, log)
    end

    for i in 1:rrt.max_iter
        # Sample a node
        new_node = if rand() < rrt.goal_sample_rate
            Node(rrt.goal.position)
        else
            sample(rrt)
        end

        # Get the nearest node
        nearest_node_index = get_nearest_node_index(rrt, new_node)
        nearest_node = rrt.nodes[nearest_node_index]

        distance_from_nearest_node = calc_distance(nearest_node, new_node)
        if rrt.step_size < distance_from_nearest_node
            new_node = get_extended_node(rrt, nearest_node, new_node)
        end

        # Check if the node is approved (e.g., if the node is inside any obstacle.)
        if !isnothing(rrt.is_approved) && !rrt.is_approved(new_node.position)
            continue
        end

        # Add the new node to the graph
        new_node.parent = nearest_node_index
        push!(rrt.nodes, new_node)

        if is_near_the_goal(rrt, new_node)
            new_node_index = length(rrt.nodes)
            rrt.goal.parent = new_node_index
            push!(rrt.nodes, rrt.goal)
            is_goaled = true
        end

        if rrt.enable_logging
            log = Dict("is_goaled" => is_goaled, "planner" => deepcopy(rrt))
            push!(logs, log)
        end

        if is_goaled
            rrt.logs = logs
            path = extract_path(rrt)
            return path
        end
    end

    rrt.logs = logs
    return Vector{Node{N}}([])
end
