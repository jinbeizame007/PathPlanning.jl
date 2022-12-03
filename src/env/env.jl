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
