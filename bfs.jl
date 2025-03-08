include("readMap.jl")
include("utils.jl")

using DataStructures
using Printf
using Plots

#different from other as BFS doesn't allow for movement cost
function randomPathBFS(filePath::String)
    myCells::Matrix{Int64} = fileToMatrixBFS(filePath)
    height, width = size(myCells)
    while !algoBFSAux((rand(1:width), rand(1:height)), (rand(1:width), rand(1:height)), myCells)
    end
end

function algoBFS(path::String, start::Tuple{Int64,Int64}, goal::Tuple{Int64,Int64})
    @printf("--------------------BFS------------------------\n")
    @printf("Finding path from %s to %s\n", start, goal)
    cells = fileToMatrixGraph(path, 5, 8)
    if !checkIfPathPossible(start, goal, cells)
        return false
    end
    @printf("Start and Goal walkable, continuing path finding\n")
    timeStamp = time()
    cheminTrouveBool, came_from, looked_at_count, added_to_queue_count =  algoBFSAux(start::Tuple{Int64,Int64}, goal::Tuple{Int64,Int64}, cells)
    doneTime = time()-timeStamp
    caseCount, path_size = showPathPlots!(cells, came_from, start, goal)
    @printf("Taille du chemin: %d\n", caseCount)
    println("Cell looked at: ", looked_at_count)
    println("Cell added to queue or value changed in queue: ", added_to_queue_count)                
    @printf("Temps utilisé: %f\n", doneTime)
end

function algoBFSAux(start::Tuple{Int64,Int64}, goal::Tuple{Int64,Int64}, cells::Matrix{Int64})::Tuple{Bool, Dict{Tuple{Int64,Int64},Tuple{Int64,Int64}}, Int64, Int64}
    toCheck = Queue{Tuple{Int64, Int64}}()
    enqueue!(toCheck, start)
    came_from = Dict{Tuple{Int64, Int64}, Tuple{Int64, Int64}}()
    came_from[start] = (0, 0) #montre que cest point de depart
    visited = Set{Tuple{Int64, Int64}}() #matrice de faux ?
    
    added_to_queue_count = 0
    looked_at_count = 0
    goalFound = false
    while !isempty(toCheck)
        looked_at_count+=1
        current_pos = dequeue!(toCheck)
        push!(visited, current_pos)
        
        if current_pos == goal
            goalFound = true                        
            break
        end

        for next in getNeighbors(current_pos, cells)
            if !haskey(came_from, next)
                added_to_queue_count +=1
                enqueue!(toCheck, next)
                came_from[next] = current_pos
            end
        end
    end
    if goalFound
        return (true, came_from, looked_at_count, added_to_queue_count)
    else
        return (false, Dict(), looked_at_count, added_to_queue_count)
    end
end


