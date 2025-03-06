include("readMap.jl")
include("utils.jl")
include("bfs.jl")
include("djikstra.jl")
include("astar.jl")
include("glouton.jl")

function testAlgorithms(path::String, start::Tuple{Int64,Int64}, goal::Tuple{Int64,Int64}) #add random possiblites
    algoBFS(path, start, goal)
    algoDjikstra(path, start, goal)
    algoAstar(path, start, goal)
    algoGlouton(path, start, goal)
end

