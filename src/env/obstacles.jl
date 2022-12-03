struct RectObstacle{N}
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
