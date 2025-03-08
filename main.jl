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

function benchAlgo(algo::Function, start::Tuple{Int64,Int64}, goal::Tuple{Int64,Int64})
        @printf("--------------------A*------------------------\n")
        @printf("Finding path from %s to %s\n", start, goal)
        cells = fileToMatrixGraph(path, 5, 8)
        if !checkIfPathPossible(start, goal, cells)
            return false
        end
        @printf("Start and Goal walkable, continuing path finding\n")
        timeStamp = time()
        cheminTrouveBool, came_from, looked_at_count, added_to_queue_count =  algoAstarAux(start::Tuple{Int64,Int64}, goal::Tuple{Int64,Int64}, cells)
        doneTime = time()-timeStamp
        path_size = showPathPlots!(cells, came_from, start, goal)
        @printf("Taille du chemin: %d\n", path_size)
        println("Cell looked at: ", looked_at_count)
        println("Cell added to queue or value changed in queue: ", added_to_queue_count)                
        @printf("Temps utilisé: %f\n", doneTime)
end
