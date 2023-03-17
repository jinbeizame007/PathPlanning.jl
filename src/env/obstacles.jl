abstract type AbstractObstacle end

"""
    RectObstacle{N}

A Rectangle-type Obstacle.

# Fields
- `center::SVector{N,Float64}`: center position of the rectangle-type obstacle.
- `size::SVector{N,Float64}`: width of each side of the rectangle-type obstacle.
"""
struct RectObstacle{N} <: AbstractObstacle
    center::SVector{N,Float64}
    size::SVector{N,Float64}

    function RectObstacle(
        center::SVector{N,Float64},
        size::SVector{N,Float64},
    ) where {N}
        if any(size .<= 0.0)
            throw(DomainError(size, "The `size` $size should be positive."))
        end
    
        return new{N}(center, size)
    end
end

"""
    is_inside(obs::RectObstacle{N}, position::SVector{N,Float64})::Bool where {N}

Return if the `position` is inside the RectObstacle.
"""
function is_inside(
    obs::RectObstacle{N},
    position::Union{SVector{2,Float64},Vector{Float64}};
    pad_size::Float64=0.0
)::Bool where {N}
    return all(obs.center - ((obs.size./2).+pad_size) .<= position .<= obs.center + ((obs.size./2).+pad_size))
end

function calc_distance(
    obs::RectObstacle{2},
    position::Union{SVector{2,Float64},Vector{Float64}};
    pad_size::Float64=0.0
)::Float64
    x_min = obs.center[1] - obs.size[1]/2 - pad_size
    x_max = obs.center[1] + obs.size[1]/2 + pad_size
    y_min = obs.center[2] - obs.size[2]/2 - pad_size
    y_max = obs.center[2] + obs.size[2]/2 + pad_size

    if position[1] < x_min
        if position[2] < y_min
            return ((x_min - position[1])^2 + (y_min - position[2])^2)^0.5
        elseif position[2] <= y_max
            return x_min - position[1]
        else
            return ((x_min - position[1])^2 + (y_max - position[2])^2)^0.5
        end
    elseif position[1] <= x_max
        if position[2] < y_min
            return y_min - position[2]
        elseif position[2] <= y_max
            return -1 * min(x_max - position[1], position[1] - x_min, y_max - position[2], position[2] - y_min)
        else
            return position[2] - y_max
        end
    else
        if position[2] < y_min
            return ((x_max - position[1])^2 + (y_min - position[2])^2)^0.5
        elseif position[2] <= y_max
            return position[1] - x_max
        else
            return ((x_max - position[1])^2 + (y_max - position[2])^2)^0.5
        end
    end
end

"""
    CircleObstacle{N}

A Circle-type Obstacle.

# Fields
- `center::SVector{N,Float64}`: center position of the circle-type obstacle.
- `radius::Float64`: radius of the circle-type obstacle.
"""
struct CircleObstacle{N} <: AbstractObstacle
    center::SVector{N,Float64}
    radius::Float64

    function CircleObstacle(
        center::SVector{N,Float64},
        radius::Float64,
    ) where {N}
        if any(radius <= 0.0)
            throw(DomainError(radius, "The `radius` $radius should be positive."))
        end
    
        return new{N}(center, radius)
    end
end

"""
    is_inside(obs::CircleObstacle{N},, position::SVector{N,Float64})::Bool where {N}

Return if the `position` is inside the CircleObstacle.
"""
function is_inside(
    obs::CircleObstacle{N},
    position::Union{SVector{2,Float64},Vector{Float64}};
    pad_size::Float64=0.0
)::Bool where {N}
    distance = sum((obs.center - position).^2.0)^0.5
    return distance <= obs.radius + pad_size
end

function calc_distance(
    obs::CircleObstacle{N},
    position::Union{SVector{2,Float64},Vector{Float64}};
    pad_size::Float64=0.0
)::Float64 where {N}
    return sum((obs.center .- position).^2)^0.5 - (obs.radius + pad_size)
end
