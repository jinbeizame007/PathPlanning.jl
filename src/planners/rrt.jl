mutable struct Node{N}
    position::SVector{N, Float64}
    parent::Union{Node{N}, Nothing}
end

function Node(position::SVector{N, Float64}) where {N}
    return Node{N}(position, nothing)
end

mutable struct RRT{N}
    start::Node{N}
    goal::Node{N}
    low::SVector{N,Float64}
    high::SVector{N,Float64}
    max_iter::Int64
end

function RRT(
    start::SVector{N,Float64},
    goal::SVector{N,Float64},
    low::SVector{N,Float64},
    high::SVector{N,Float64};
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

    return RRT{N}(Node(start), Node(goal), low, high, max_iter)
end
