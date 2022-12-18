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
https://user-images.githubusercontent.com/16977484/208250130-c7edf1b8-8a0c-4288-a524-b939eeaf6e39.mp4

## RRT-Connect
https://user-images.githubusercontent.com/16977484/208293176-51a4fe1f-3628-4933-82a4-291e9f5bf78f.mp4

## RRT*
https://user-images.githubusercontent.com/16977484/208250144-56b4238a-b468-467e-af81-bf964e7254c1.mp4
