mutable struct Node{N}
    position::SVector{N, Float64}
    parent::Union{Node{N}, Nothing}
end

function Node(position::SVector{N, Float64}) where {N}
    return Node{N}(position, nothing)
end
