using Documenter, PathPlanning

makedocs(;
    sitename="PathPlanning.jl",
    pages = [
        "Index"=>"index.md",
        "Planners"=>[
            "RRT" => "planners/rrt.md",
        ],
    ]
)
