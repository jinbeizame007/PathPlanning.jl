abstract type AbstractTrajOpt{N} <: AbstractPlanner{N} end

const FINITE_DIFF_RULE_LENGTH = 7

FINITE_CENTRAL_DIFF_COEFFS = [
  0.0 0.0 0.0 1.0 0.0 0.0 0.0;
  0.0 1.0/12.0 -2.0/3.0 0.0 2.0/3.0 -1.0/12.0 0.0;
  0.0 -1/12.0 16.0/12.0 -30.0/12.0 16.0/12.0 -1.0/12.0 0.0;
  0.0 1.0/12.0 -17.0/12.0 46.0/12.0 -46.0/12.0 17.0/12.0 -1.0/12.0
]

mutable struct STOMP{N} <: AbstractTrajOpt{N}
    start::SVector{N,Float64}
    goal::SVector{N,Float64}
    low::SVector{N,Float64}
    high::SVector{N,Float64}
    cost_func::Union{Nothing, Function}
    num_samples::Int64
    path_length::Int64
    dist::MvNormal{Float64}
    mean::Matrix{Float64}
    M::Matrix{Float64}
    enable_logging::Bool
    logs::Vector{Dict{String, Any}}
end

function STOMP(
    start::SVector{N,Float64},
    goal::SVector{N,Float64},
    low::SVector{N,Float64},
    high::SVector{N,Float64};
    cost_func::Union{Function, Nothing}=nothing,
    num_samples::Int64 = 50,
    path_length::Int64 = 30,
    dt::Float64 = 0.05,
    enable_logging::Bool = false
) where N
    mean = linear_interpolation(start, goal, path_length)

    path_length_padded = path_length + 2 * (FINITE_DIFF_RULE_LENGTH - 1)
    A_padded = create_finite_difference_matrix(path_length_padded)
    R_padded = dt .* (transpose(A_padded) * A_padded)
    R = R_padded[
        FINITE_DIFF_RULE_LENGTH:(FINITE_DIFF_RULE_LENGTH+path_length-1),
        FINITE_DIFF_RULE_LENGTH:(FINITE_DIFF_RULE_LENGTH+path_length-1)
    ]
    R_inv = inv(R)

    n = size(R_inv,1)
    M = Matrix{Float64}(undef, n, n)
    M .= R_inv
    for i in 1:n
        M[:,i] .*= 1.0 / (path_length * M[i,i])
    end

    dist = MvNormal(zeros(path_length), Symmetric(R_inv))

    logs = Vector{Dict{String, Any}}([])
    return STOMP{N}(start, goal, low, high, cost_func, num_samples, path_length, dist, mean, M, enable_logging, logs)
end

function linear_interpolation(start::SVector{N,Float64}, goal::SVector{N,Float64}, path_length::Int64)::Matrix{Float64} where N
    path = zeros(N, path_length)
    for i in 1:N
        path[i,:] .= Vector{Float64}(range(start[i], goal[i]; length=path_length))
    end
    return path
end

function create_finite_difference_matrix(path_length::Int64; dt::Float64=1.0, order::Int64=2)::Matrix{Float64}
    A = Matrix{Float64}(I, path_length, path_length)
    multiplier = 1.0 / (dt^order)
    half_finite_diff_rule_length = div(FINITE_DIFF_RULE_LENGTH, 2)

    for i in 0:path_length-1
        for j in -half_finite_diff_rule_length:half_finite_diff_rule_length
            index = i + j

            if index < 0 || path_length <= index
                continue
            end

            A[1+i,1+index] = multiplier * FINITE_CENTRAL_DIFF_COEFFS[1 + order, 1 + j + half_finite_diff_rule_length]
        end
    end
    return A
end

function sample(stomp::STOMP{N}) where N
    noise = zeros(N, stomp.path_length, stomp.num_samples)
    samples = rand(stomp.dist, stomp.num_samples * N)
    for i in 1:stomp.num_samples*N
        noise[1+(i-1)%N, :, 1+floor(Int64, (i-1)/N)] .= samples[:,i]
    end
    return 0.05 .* noise
end

function clip_noise!(stomp::STOMP{N}, mean::Matrix{Float64}, noises::Array{Float64}) where N
    for d in 1:N
        max_noises = stomp.high[d] .- mean[d,:]
        min_noises = stomp.low[d] .- mean[d,:]
        for t in 1:stomp.path_length, i in 1:stomp.num_samples
            noises[d,t,i] = clamp(noises[d,t,i], min_noises[t], max_noises[t])
        end
    end
end

function calc_distance_cost(stomp::STOMP{N}, env::Env, position::Vector{Float64}; pad_size::Float64=0.0) where N
    min_distance = minimum([Envs.calc_distance(obs, position; pad_size=pad_size) for obs in env.obstacles])
    return -1 * min(min_distance, 0.0)
end

function calc_distance_cost(stomp::STOMP{N}, env::Env, paths::Array{Float64}) where N
    distance_costs = Matrix{Float64}(undef, stomp.path_length, stomp.num_samples)
    for n in 1:stomp.num_samples, t in 1:stomp.path_length
        distance_costs[t,n] = calc_distance_cost(stomp, env, paths[:,t,n])
    end
    return distance_costs
end

function calc_torque_cost(stomp::STOMP{N}, paths::Array{Float64}) where N
    torque_cost = zeros(stomp.path_length, stomp.num_samples)
    torque_cost[2:end-1,:] += sum(abs.(paths[:,2:end-1,:] - paths[:,1:end-2,:]), dims=1)[1,:,:]
    torque_cost[2:end-1,:] += sum(abs.(paths[:,3:end,:] - paths[:,2:end-1,:]), dims=1)[1,:,:]
    return torque_cost
end

function calc_probabilities(stomp::STOMP{N}, costs::Matrix{Float64}; h::Float64=10.0, epsilon::Float64=1e-2)::Matrix{Float64} where N
    exponentials = eps.(-h .* (costs .- minimum(costs)) ./ (max.(maximum(costs) - minimum(costs), epsilon)))
    probabilities = exponentials ./ sum(exponentials, dims=2)
    return probabilities
end

function plan(stomp::STOMP{N}) where N
    logs = Vector{Dict{String, Any}}([])

    for _ in 1:50
        mean = stomp.mean
        noises = sample(stomp)
        clip_noise!(stomp, mean, noises)
        paths = Array{Float64}(undef, N, stomp.path_length, stomp.num_samples)
        for i in 1:stomp.num_samples
            paths[:,:,i] = mean + noises[:,:,i]
        end

        if stomp.enable_logging
            log = Dict("mean" => deepcopy(stomp.mean), "sampled_paths" => paths)
            push!(logs, log)
        end
        
        # (path_length, num_samples)
        costs = stomp.cost_func(stomp, paths)
        # (path_length, num_samples)
        probabilities = calc_probabilities(stomp, costs)

        deltas = zeros(N, stomp.path_length)
        for d in 1:N, n in 1:stomp.num_samples
            deltas[d,:] .+= probabilities[:,n] .* noises[d,:,n]
        end

        for d in 1:N
            stomp.mean[d,:] .-= stomp.M * deltas[d,:]
        end
    end

    stomp.logs = logs
    return stomp.mean
end
