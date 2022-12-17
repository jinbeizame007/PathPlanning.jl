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
"""
mutable struct RRTStar{N} <: AbstractRRT{N}
    start::Node{N}
    goal::Node{N}
    low::SVector{N,Float64}
    high::SVector{N,Float64}
    nodes::Vector{Node{N}}
    goal_sample_rate::Float64
    step_size::Float64
    max_iter::Int64
    is_approved::Union{Function, Nothing}
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

    return RRTStar{N}(Node(start), Node(goal), low, high, nodes, goal_sample_rate, step_size, max_iter, is_approved)
end

function get_near_node_indices(rrt::RRTStar{N}, new_node::Node{N}, distance::Float64)::Vector{Int64} where {N}
    return Vector{Int64}([i for i in 1:length(rrt.nodes) if calc_distance(rrt.nodes[i], new_node) <= distance])
end

function rewire_near_nodes!(rrt_star::RRTStar{N}, near_node_indices::Vector{Int64}, new_node_index::Int64) where {N}
    new_node = rrt_star.nodes[new_node_index]
    for near_node_index in near_node_indices
        near_node = rrt_star.nodes[near_node_index]

        new_cost = new_node.cost + calc_distance(new_node, near_node)
        if new_cost < near_node.cost
            near_node.parent = new_node_index
            near_node.cost = new_cost
        end
    end

    return nothing
end

"""
    plan(rrt::RRT{N})::Vector{Node{N}} where {N}

Find and return a path from the start node `rrt.start` to the goal node `rrt.goal` with RRT algorithm.

# Arguments
- rrt::RRT{N}: a planner using RRT

# Returns
- `path::Vector{Node{N}}`: path (sequence of nodes) from the start node `rrt.start` to the goal node `rrt.goal`
"""
function plan(rrt_star::RRTStar{N})::Vector{Node{N}} where {N}
    rrt_star.nodes = [rrt_star.start]
    is_goaled = false

    for i in 1:rrt_star.max_iter
        # Sample a node
        new_node = if rand() < rrt_star.goal_sample_rate
            Node(rrt_star.goal.position)
        else
            sample(rrt_star)
        end

        # Get the nearest node
        nearest_node_index = get_nearest_node_index(rrt_star, new_node)
        nearest_node = rrt_star.nodes[nearest_node_index]

        distance_from_nearest_node = calc_distance(nearest_node, new_node)
        if rrt_star.step_size < distance_from_nearest_node
            new_node = get_extended_node(rrt_star, nearest_node, new_node)
        end

        # Check if the node is approved (e.g., if the node is inside any obstacle.)
        if !isnothing(rrt_star.is_approved) && !rrt_star.is_approved(new_node.position)
            continue
        end

        # Add the new node to the graph
        new_node.parent = nearest_node_index
        push!(rrt_star.nodes, new_node)
        new_node_index = length(rrt_star.nodes)

        if !is_goaled && is_near_the_goal(rrt_star, new_node)
            new_node_index = length(rrt_star.nodes)
            rrt_star.goal.parent = new_node_index
            is_goaled = true
        end

        num_nodes = length(rrt_star.nodes)
        distance_threshold = 50.0 * (log(num_nodes) / num_nodes)^(1/N)
        near_node_indices = get_near_node_indices(rrt_star, new_node, distance_threshold)
        rewire_near_nodes!(rrt_star, near_node_indices, new_node_index)
    end

    return extract_path(rrt_star)
end