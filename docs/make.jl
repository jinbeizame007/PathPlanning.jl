using Documenter, PathPlanning

makedocs(;
    sitename="PathPlanning.jl",
    pages = [
        "Index"=>"index.md",
        "Planners"=>[
            "RRT" => "planners/rrt.md",
        ],
        "Envs"=>[
            "Env" => "envs/env.md",
        ],
    ]
)
deploydocs(
    repo = "github.com/jinbeizame007/PathPlanning.jl.git";
    push_preview = true
)
