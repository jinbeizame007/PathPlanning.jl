const FINITE_DIFF_RULE_LENGTH = 7

FINITE_CENTRAL_DIFF_COEFFS = [
  0.0 0.0 0.0 1.0 0.0 0.0 0.0;
  0.0 1.0/12.0 -2.0/3.0 0.0 2.0/3.0 -1.0/12.0 0.0;
  0.0 -1/12.0 16.0/12.0 -30.0/12.0 16.0/12.0 -1.0/12.0 0.0;
  0.0 1.0/12.0 -17.0/12.0 46.0/12.0 -46.0/12.0 17.0/12.0 -1.0/12.0
]

mutable struct STOMP{N}
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
end

function STOMP(
    start::SVector{N,Float64},
    goal::SVector{N,Float64},
    low::SVector{N,Float64},
    high::SVector{N,Float64};
    cost_func::Union{Function, Nothing}=nothing,
    num_samples::Int64 = 10,
    path_length::Int64 = 5,
    dt::Float64 = 1.0
) where N
    mean = linear_interpolation(start, goal, path_length)

    start_index_padded = FINITE_DIFF_RULE_LENGTH
    path_length_padded = path_length + 2 * (FINITE_DIFF_RULE_LENGTH - 1)
    A_padded = create_finite_difference_matrix(path_length_padded)
    R_padded = dt .* transpose(A_padded) * A_padded
    R = R_padded[
        FINITE_DIFF_RULE_LENGTH:(FINITE_DIFF_RULE_LENGTH+path_length-1),
        FINITE_DIFF_RULE_LENGTH:(FINITE_DIFF_RULE_LENGTH+path_length-1)
    ]
    R_inv = pinv(R)

    n = size(R_inv,1)
    M = Matrix{Float64}(undef, n, n)
    for i in 1:n
        M[:,i] .= R_inv[:,i] ./ maximum(R_inv[:,i]) ./ n
    end

    dist = MvNormal(zeros(path_length), Symmetric(R_inv))
    return STOMP{N}(start, goal, low, high, cost_func, num_samples, path_length, dist, mean, M)
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
    multiplier = 1.0 / dt^order
    half_finite_diff_rule_length = floor(Int64, FINITE_DIFF_RULE_LENGTH/2)

    for i in 1:path_length
        for j in -half_finite_diff_rule_length:half_finite_diff_rule_length
            index = i + j

            if index < 1
                index = 0
                continue
            end

            if path_length < index
                index = path_length
                continue
            end

            A[i,index] = multiplier * FINITE_CENTRAL_DIFF_COEFFS[1 + order, 1 + j + half_finite_diff_rule_length]
        end
    end
    return A
end

function create_inverse_positive_semi_definite_matrix(A::Matrix{Float64})::Matrix{Float64}
    R_inv = inv(A * A)
    return R_inv
end

function sample(stomp::STOMP{N}) where N
    noise = zeros(N, stomp.path_length, stomp.num_samples)
    samples = rand(stomp.dist, stomp.num_samples * N)
    for i in 1:stomp.num_samples*N
        noise[1+(i-1)%N, :, 1+floor(Int64, (i-1)/N)] .= samples[:,i]
    end
    return noise
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

# function calc_cost(stomp::STOMP{N}, paths::Matrix{Float64}) where N
    
# end

function calc_probabilities(stomp::STOMP{N}, costs::Matrix{Float64}; h::Float64=10.0)::Matrix{Float64} where N
    exponentials = Matrix{Float64}(undef, stomp.path_length, stomp.num_samples)
    min_costs = minimum(costs, dims=1)
    max_costs = maximum(costs, dims=1)
    
    for n in 1:stomp.num_samples, t in 1:stomp.path_length
        exponentials[t,n] = eps(-h * (costs[t,n] - min_costs[n]) / (max_costs[n] - min_costs[n]))
    end

    probabilities = exponentials ./ sum(exponentials, dims=1)
    return probabilities
end

function plan(stomp::STOMP{N}) where N
    mean = stomp.mean
    noises = sample(stomp)
    paths = Array{Float64}(undef, N, stomp.path_length, stomp.num_samples)
    for i in 1:stomp.num_samples
        paths[:,:,i] = mean + noises[:,:,i]
    end
    
    costs = stomp.cost_func(stomp, paths)
    probabilities = calc_probabilities(stomp, costs)

    deltas = zeros(N, stomp.path_length)
    for d in 1:N, n in 1:stomp.num_samples
        deltas[d,:] .+= probabilities[:,n] .* noises[d,:,n]
    end

    for d in 1:N
        mean[d,:] .= stomp.M * deltas[d,:]
    end
end
