# PathPlanning.jl
[![CI](https://github.com/jinbeizame007/PathPlanning.jl/actions/workflows/ci.yml/badge.svg)](https://github.com/jinbeizame007/PathPlanning.jl/actions/workflows/ci.yml)
[![Global Docs](https://github.com/jinbeizame007/PathPlanning.jl/actions/workflows/documantation.yaml/badge.svg)](https://jinbeizame007.github.io/PathPlanning.jl/dev/)

*PathPlanning.jl* is a framework for finding a path for robot or an agent to follow in order to reach a specific goal or detination. *PathPlanning.jl* offers high modularity and allows users to easily incorporate various constraints, such as obstacles or inverse kinematics, through the use of user-defined functions that determine whether or not a given waypoint is approved. This makes it a versatile tool for addressing a wide range of path planning problems.

# Example

```Julia
using PathPlanning.Planners
using PathPlanning.Envs
using PathPlanning.Plot
using StaticArrays

# Set bounds of the search space
low = SA[0.0, 0.0]
high = SA[50.0, 30.0]

# Set start and goal position
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
planner = RRTStar(start, goal, low, high; step_size=1.5, max_iter=2000, is_approved=is_approved)
path = plan(planner)

# Visualize
animate(env, planner; framerate=60)
```

# Animations
## RRT
https://user-images.githubusercontent.com/16977484/208250130-c7edf1b8-8a0c-4288-a524-b939eeaf6e39.mp4

## RRT-Connect
https://user-images.githubusercontent.com/16977484/208293176-51a4fe1f-3628-4933-82a4-291e9f5bf78f.mp4

## RRT*
https://user-images.githubusercontent.com/16977484/208668336-cdc4b867-4384-4b9a-ad63-775490e73389.mp4

## Informed-RRT*
https://user-images.githubusercontent.com/16977484/208668385-1ca719ff-1f63-4332-9ac2-f62ca66764d6.mp4

## STOMP (Stochastic Trajectory Optimization for Motion Planning)
https://user-images.githubusercontent.com/16977484/225943486-47a76743-0149-4813-8342-8cacccd4a7eb.mp4
