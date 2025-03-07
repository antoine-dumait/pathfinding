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
    # timeStamp = time()
    algoBFSAux(start::Tuple{Int64,Int64}, goal::Tuple{Int64,Int64}, fileToMatrixBFS(path))
    # @printf("Temps utilisé: %f\n", time()-timeStamp)
end

function algoBFSAux(start::Tuple{Int64,Int64}, goal::Tuple{Int64,Int64}, cells::Matrix{Int64})::Bool
    timeStamp = time()
    
    @printf("--------------------BFS------------------------\n")
    @printf("Finding path from %s to %s\n", start, goal)
    
    if !checkIfPathPossible(start, goal, cells)
        return false
    end

    toCheck = Queue{Tuple{Int64, Int64}}()
    enqueue!(toCheck, start)
    came_from = Dict{Tuple{Int64, Int64}, Tuple{Int64, Int64}}()
    came_from[start] = (0, 0) #montre que cest point de depart
    visited = Set{Tuple{Int64, Int64}}() #matrice de faux ?
    
    added_to_queue_count = 0
    looked_at_count = 0
    while !isempty(toCheck)
        looked_at_count+=1
        current_pos = dequeue!(toCheck)
        push!(visited, current_pos)
        
        if current_pos == goal
            @printf("Temps utilisé: %d\n", timeStamp-time())
            # showPath(came_from, start, goal, cells)
            showPathPlots!(cells, came_from, start, goal)
            println("Cell looked at: ", looked_at_count)
            println("Cell added to queue or value changed in queue: ", added_to_queue_count)                        
            return true
        end

        for next in getNeighbors(current_pos, cells)
            if !haskey(came_from, next)
                added_to_queue_count +=1
                enqueue!(toCheck, next)
                came_from[next] = current_pos
            end
        end
    end

    @printf("No path found between %s and %s\n", start, goal)
    return false
end


