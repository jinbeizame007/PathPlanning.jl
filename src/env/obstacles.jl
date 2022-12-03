abstract type AbstractObstacle end

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

function is_inside(
    obs::RectObstacle{N},
    position::SVector{N,Float64},
)::Bool where {N}
    return all(obs.center - obs.size./2 .<= position .<= obs.center + obs.size./2)
end

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

function is_inside(
    obs::CircleObstacle{N},
    position::SVector{N,Float64},
)::Bool where {N}
    distance = sum((obs.center - position).^2.0)^0.5
    return distance <= obs.radius
end

function create_example_2D_env()::Env
    obstacles = [
        RectObstacle(SA[18.0, 13.0], SA[8.0, 2.0]),
        RectObstacle(SA[22.0, 23.5], SA[8.0, 3.0]),
        RectObstacle(SA[27.0, 13.0], SA[2.0, 12.0]),
        RectObstacle(SA[37.0, 15.0], SA[10.0, 2.0]),
        CircleObstacle(SA[7.0, 12.0], 3.0),
        CircleObstacle(SA[46.0, 20.0], 2.0),
        CircleObstacle(SA[15.0, 5.0], 2.0),
        CircleObstacle(SA[37.0, 7.0], 3.0),
        CircleObstacle(SA[37.0, 23.0], 3.0),
    ]
    return Env(obstacles)
end
