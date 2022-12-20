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
    position::SVector{N,Float64},
)::Bool where {N}
    return all(obs.center - obs.size./2 .<= position .<= obs.center + obs.size./2)
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
    position::SVector{N,Float64},
)::Bool where {N}
    distance = sum((obs.center - position).^2.0)^0.5
    return distance <= obs.radius
end
