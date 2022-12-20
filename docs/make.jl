using Documenter, PathPlanning

makedocs(;
    sitename="PathPlanning.jl",
    pages = [
        "Index"=>"index.md",
        "Planners"=>[
            "Planners" => "planners/planners.md",
        ],
        "Envs"=>[
            "Env" => "envs/envs.md",
        ],
        "Plot"=>[
            "Env" => "plot/plot.md",
        ],
    ]
)
deploydocs(
    repo = "github.com/jinbeizame007/PathPlanning.jl.git";
    push_preview = true
)
