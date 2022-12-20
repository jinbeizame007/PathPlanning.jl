"""
    InformedRRTStar{N}

A planner using Informed RRT*.

# Fields
- `start::Node{N}`: start position in the search space
- `goal::Node{N}`: goal position in the search space
- `center::SVector{N,Float64}`: center position between the start node and the goal node
- `low::SVector{N,Float64}`: lower bounds of the search space
- `high::SVector{N,Float64}`: upper bounds of the search space
- `cost_min::Float64`: the minimum cost (cost if the start node and the goal node are linear interpolated)
- `cost_max::Union{Float64,Nothing}`: the maximum cost (cost of the current path that the planner found)
- `rotation_matrix::SMatrix{N,N,Float64}`: rotation matrix to rotate to the world frame (same as `C` in the original paper)
- `nodes::Vector{Node{N}}`: nodes configure tje graph
- `goal_sample_rate::Float64`: rate of sampling the goal node
- `step_size::Float64`: maximum distance between each node
- `max_iter::Int64`: maximum the number of the iterations
- `is_approved::Union{Function, Nothing}`: a function that returns if the node can be added the graph
- `enable_logging::Bool`: whether to log (is_goaled, RRTStar instance) for each steps in planning
- `logs::Vector{Dict{String, Any}}`: (is_goaled, RRTStar instance) for each steps in planning
"""
mutable struct InformedRRTStar{N} <: AbstractRRTStar{N}
    start::Node{N}
    goal::Node{N}
    center::SVector{N,Float64}
    low::SVector{N,Float64}
    high::SVector{N,Float64}
    cost_min::Float64
    cost_max::Union{Float64,Nothing}
    rotation_matrix::SMatrix{N,N,Float64}
    nodes::Vector{Node{N}}
    goal_sample_rate::Float64
    step_size::Float64
    max_iter::Int64
    is_approved::Union{Function, Nothing}
    enable_logging::Bool
    logs::Vector{Dict{String, Any}}
end

"""
    InformedRRTStar(
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

Constructor of InformedRRTStar struct.
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
function InformedRRTStar(
    start::SVector{N,Float64},
    goal::SVector{N,Float64},
    low::SVector{N,Float64},
    high::SVector{N,Float64};
    goal_sample_rate::Float64 = 0.2,
    step_size::Union{Float64,Nothing} = nothing,
    max_iter::Int64 = 500,
    is_approved::Union{Function, Nothing} = nothing,
    enable_logging::Bool = false
)::InformedRRTStar{N} where {N}
    if any(high .<= low)
        throw(DomainError((low, high), "The `low` ($low) should be lower than `high` ($high)."))
    end

    if !all(low .<= start .<= high)
        throw(DomainError(start, "The `start` ($start) should be between `low` ($low) and `high` ($high)."))
    end

    if !all(low .<= goal .<= high)
        throw(DomainError(goal, "The `goal` ($goal) should be between `low` ($low) and `high` ($high)."))
    end

    center = (start + goal) ./ 2.0
    cost_min = sum((goal - start).^2.0)^0.5
    cost_max = nothing
    rotation_matrix = get_rotation_to_world_frame(start, goal)

    nodes = Vector{Node{N}}([])
    if isnothing(step_size)
        step_size = sum((goal - start).^2.0)^0.5 / 20.0
    end

    logs = Vector{Dict{String, Any}}([])
    return InformedRRTStar{N}(Node(start), Node(goal), center, low, high, cost_min, cost_max, rotation_matrix,
        nodes, goal_sample_rate, step_size, max_iter, is_approved, enable_logging, logs)
end

function get_rotation_to_world_frame(start::SVector{N,Float64}, goal::SVector{N,Float64})::SMatrix{N,N,Float64} where N
    a_1 = (goal - start) ./ sum((goal - start).^2).^0.5
    e_1 = zeros(N)
    e_1[1] = 1.0
    M = a_1 * e_1'

    F = svd(M; full=true)
    D = Matrix{Float64}(I, N, N)
    D[end-1,end-1] = det(F.U)
    D[end,end] = det(F.V)

    C = F.U * D * F.Vt
    return SMatrix{N,N,Float64}(C)
end

function sample_from_unit_ball(planner::InformedRRTStar{N})::SVector{N,Float64} where {N}
    while true
        position = @SVector rand(N)
        position = position .* 2.0 .- 1.0
        if sum(position.^2.0) <= 1
            return position
        end
    end
end

function sample_from_informed_elipse(planner::InformedRRTStar{N})::Node{N} where {N}
    center = planner.center
    cost_min = planner.cost_min
    cost_max = planner.cost_max
    C = planner.rotation_matrix

    radiuses = zeros(N)
    radiuses[1] = cost_max / 2.0
    radiuses[2:end] .= (cost_max^2 - cost_min^2)^0.5 / 2.0
    L = Diagonal(radiuses)

    while true
        position = sample_from_unit_ball(planner)
        position = C * L * position + center
        
        if all(planner.low .<= position .<= planner.high)
            return Node(SVector{N}(position))
        end
    end
end

"""
    plan(planner::InformedRRTStar{N})::Vector{Node{N}} where {N}

Find and return a path from the start node `planner.start` to the goal node `planner.goal` with Informed RRT* algorithm.

# Arguments
- planner::InformedRRTStar{N}: a planner using RRT*

# Returns
- `path::Vector{Node{N}}`: path (sequence of nodes) from the start node `rrt.start` to the goal node `rrt.goal`
"""
function plan(planner::InformedRRTStar{N})::Vector{Node{N}} where {N}
    planner.nodes = [planner.start]
    is_goaled = false

    logs = Vector{Dict{String, Any}}([])
    if planner.enable_logging
        log = Dict("is_goaled" => is_goaled, "planner" => deepcopy(planner))
        push!(logs, log)
    end

    for i in 1:planner.max_iter
        # Sample a node
        new_node = if is_goaled
            #sample(planner)
            sample_from_informed_elipse(planner)
        else
            if rand() < planner.goal_sample_rate
                Node(planner.goal.position)
            else
                sample(planner)
            end
        end

        # Get the nearest node
        nearest_node_index = get_nearest_node_index(planner.nodes, new_node)
        nearest_node = planner.nodes[nearest_node_index]

        # Get a node extended from the nearest node
        distance_from_nearest_node = calc_distance(nearest_node, new_node)
        if planner.step_size < distance_from_nearest_node
            new_node = get_extended_node(planner, nearest_node, new_node)
        end

        # Check if the node is approved (e.g., if the node is inside any obstacle.)
        if !isnothing(planner.is_approved) && !planner.is_approved(new_node.position)
            continue
        end

        # Add the new node to the graph
        parent_node_index = get_shortest_parent_node_index(planner, new_node)
        add_node!(planner, new_node, parent_node_index)
        new_node_index = length(planner.nodes)

        # Add goal node to the graph if the new node is near the goal node
        if !is_goaled && is_near_the_goal(planner, new_node)
            new_node_index = length(planner.nodes)
            add_node!(planner, planner.goal, new_node_index)
            is_goaled = true
        end

        # Rewire near nodes
        near_node_indices = get_near_node_indices(planner, new_node, planner.step_size)
        rewire_near_nodes!(planner, near_node_indices, new_node_index)

        if is_goaled
            planner.cost_max = planner.goal.cost
        end

        if planner.enable_logging
            log = Dict("is_goaled" => is_goaled, "planner" => deepcopy(planner))
            push!(logs, log)
        end
    end

    planner.logs = logs
    return extract_path(planner)
end
