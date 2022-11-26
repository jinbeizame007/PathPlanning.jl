mutable struct Node{N}
    position::SVector{N, Float64}
    parent::Union{Int64, Nothing}
end

function Node(position::SVector{N, Float64}) where {N}
    return Node{N}(position, nothing)
end

function calc_distance(node1::Node{N}, node2::Node{N})::Float64 where {N}
    return sum((node1.position - node2.position).^2.0)^0.5
end

mutable struct RRT{N}
    start::Node{N}
    goal::Node{N}
    low::SVector{N,Float64}
    high::SVector{N,Float64}
    nodes::Vector{Node{N}}
    step_size::Float64
    max_iter::Int64
end

function RRT(
    start::SVector{N,Float64},
    goal::SVector{N,Float64},
    low::SVector{N,Float64},
    high::SVector{N,Float64};
    step_size::Union{Float64,Nothing}=nothing,
    max_iter::Int64 = 500
) where {N}
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
        step_size = sum((goal - start).^2.0)^0.5 / 50.0
    end

    return RRT{N}(Node(start), Node(goal), low, high, nodes, step_size, max_iter)
end

function sample(rrt::RRT{N})::Node{N} where {N}
    return Node((@SVector rand(N)) .* (rrt.high - rrt.low) - rrt.low)
end

function get_nearest_node_index(rrt::RRT{N}, new_node::Node{N})::Int64 where {N}
    distances = [calc_distance(node, new_node) for node in rrt.nodes]
    _, index = findmin(distances)
    return index
end

function get_extended_node(rrt::RRT{N}, nearest_node::Node{N}, new_node::Node{N})::Node{N} where {N}
    delta = nearest_node.position - new_node.position
    distance = calc_distance(nearest_node, new_node)
    
    position = nearest_node.position + delta * (rrt.step_size / distance)
    extended_node = Node(position)
    return extended_node
end

function is_near_the_goal(rrt::RRT{N}, node::Node{N}) where {N}
    distance_from_goal = calc_distance(rrt.goal, node)
    return distance_from_goal <= rrt.step_size
end

function extract_path(rrt::RRT{N}) where N
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

function plan(
    rrt::RRT{N}
)::Vector{Node{N}} where {N}
    rrt.nodes = [rrt.start]

    for i in 1:rrt.max_iter
        # Sample a node
        new_node = sample(rrt)

        # Get the nearest node
        nearest_node_index = get_nearest_node_index(rrt, new_node)
        nearest_node = rrt.nodes[nearest_node_index]

        distance_from_nearest_node = calc_distance(nearest_node, new_node)
        if rrt.step_size < distance_from_nearest_node
            new_node = get_extended_node(rrt, nearest_node, new_node)
        end

        new_node.parent = nearest_node_index
        push!(rrt.nodes, new_node)

        if is_near_the_goal(rrt, new_node)
            new_node_index = length(rrt.nodes)
            rrt.goal.parent = new_node_index

            path = extract_path(rrt)
            return path
        end
    end

    return Vector{Node{N}}([])
end
