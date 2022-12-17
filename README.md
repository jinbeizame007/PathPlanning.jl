# PathPlanning
[![CI](https://github.com/jinbeizame007/PathPlanning.jl/actions/workflows/ci.yml/badge.svg)](https://github.com/jinbeizame007/PathPlanning.jl/actions/workflows/ci.yml)
[![Global Docs](https://github.com/jinbeizame007/PathPlanning.jl/actions/workflows/documantation.yaml/badge.svg)](https://jinbeizame007.github.io/PathPlanning.jl/dev/)

# Example

```Julia
using PathPlanning.Planners
using PathPlanning.Envs
using PathPlanning.Plot
using StaticArrays

# Set bounds of the search space
low = SA[0.0, 0.0]
high = SA[50.0, 30.0]

# Set start and goal
start = SA[1.0, 1.0]
goal = SA[48.0, 25.0]

# Set obstacles
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
env = Env(obstacles)

# Set function to return if given node is approved
function is_approved(position::SVector{2,Float64})
    return !is_inside_any_obstacle(env, position)
end

# Plan a path
rrt = RRT(start, goal, low, high; step_size=1.0, max_iter=1000, is_approved=is_approved)
path = plan(rrt)

# Visualize
plot(env, rrt)
```

# Animations
## RRT
https://user-images.githubusercontent.com/16977484/206210649-2ae79e5b-d641-4f1f-b8b6-40edd5a7aea1.mp4
