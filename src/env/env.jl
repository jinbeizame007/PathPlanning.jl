mutable struct Env
    num_obstacles::Int64
    obstacles::Vector{AbstractObstacle}
end

function Env()
    return Env(0, Vector{AbstractObstacle}([]))
end

function Env(obstacles::Vector{AbstractObstacle})
    return Env(length(obstacles), obstacles)
end

function add_obstacle!(env::Env, obstacle::AbstractObstacle)
    env.num_obstacles += 1
    push!(env.obstacles, obstacle)
    return nothing
end

function is_inside_any_obstacle(
    env::Env,
    position::SVector{N,Float64}
)::Bool where {N}
    return any([is_inside(obstacle, position) for obstacle in env.obstacles])
end
