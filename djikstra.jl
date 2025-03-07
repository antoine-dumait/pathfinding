include("readMap.jl")
include("utils.jl")

using DataStructures
using Printf
using Plots

function randomPathDjikstra(filePath::String)
    myCells::Matrix{Int64} = fileToMatrixGraph(filePath, 5, 8)
    height, width = size(myCells)
    while !algoDjikstraAux((rand(1:width), rand(1:height)), (rand(1:width), rand(1:height)), myCells)
    end
end

function showDistAndPath(distances::Matrix{Int64}, path::Dict{Tuple{Int64,Int64},Tuple{Int64,Int64}}, start::Tuple{Int64,Int64}, goal::Tuple{Int64,Int64}, cells::Matrix{Int64})
    path_size = 1
    prev = goal
    while haskey(path, prev) && path[prev] != start
        # @show(path_size)
        path_size += cells[prev[2], prev[1]]
        prev = path[prev]
    end    
    distances2 = map(x-> x != typemax(Int64) ? x : -5, distances)
    prev = goal
    while haskey(path, prev)
        distances2[prev[2], prev[1]] = -10
        prev = path[prev]
    end
    p = heatmap(distances2, yaxis=:flip)
    @printf("Taille du chemin: %d\n", path_size)

    display(p)
end

function algoDjikstra(path::String, start::Tuple{Int64,Int64}, goal::Tuple{Int64,Int64})
    # timeStamp = time()
    @printf("--------------------Djikstra------------------------\n")
    @printf("getPath from %s to %s\n", start, goal)
    cells = fileToMatrixGraph(path, 5, 8)
    if !checkIfPathPossible(start, goal, cells)
        return false
    end
    @printf("Start and Goal walkable, continuing path finding\n")
    algoDjikstraAux(start::Tuple{Int64,Int64}, goal::Tuple{Int64,Int64}, cells)
    # showDistAndPath(distance, came_from, start, goal)
    # println("Cell looked at: ", looked_at_count)
    # println("Cell added to queue or value changed in queue: ", added_to_queue_count)                

    # @printf("Temps utilisé: %f\n", time()-timeStamp)
end

#CheminTrouveBool, Path, LookedAtCount; QueuedCount
function algoDjikstraAux(start::Tuple{Int64,Int64}, goal::Tuple{Int64,Int64}, cells::Matrix{Int64})
    # ::(Bool, Dict{Tuple{Int64,Int64},Tuple{Int64,Int64}},Matrix{Int64}, Int64, Int64)
    timeStamp = time()
    

    inf = typemax(Int64) 
    height, width = size(cells)
    
    distance = fill(inf, height, width)
    came_from = Dict{Tuple{Int64,Int64},Tuple{Int64,Int64}}()
    isDone = falses(height, width)
    
    heap = BinaryMinHeap{Tuple{Int64,Int64,Int64}}()
    push!(heap, (0, start[1], start[2]))  # distance, x,y 
    
    distance[start[2], start[1]] = 0
    
    looked_at_count = 0
    added_to_queue_count = 0
    
    while !isempty(heap)
        dist, x, y = pop!(heap)
        looked_at_count+=1
        isDone[y, x] = true
        
        if (x,y) == goal
            @printf("Temps utilisé: %d\n", timeStamp-time())
            break
        end
        
        for (nx, ny) in getNeighbors((x, y), cells)
            if !isDone[ny, nx] && cells[ny, nx] != -1  #verifie si accesible
                new_dist = dist + cells[ny, nx]
                if new_dist < distance[ny, nx]
                    added_to_queue_count+=1
                    distance[ny, nx] = new_dist
                    push!(heap, (new_dist, nx, ny))
                    came_from[(nx, ny)] = (x, y)
                end
            end
        end
    end
    
    if distance[goal[2],goal[1]] == inf
        return false
    end
    showDistAndPath(distance, came_from, start, goal, cells)
    println("Cell looked at: ", looked_at_count)
    println("Cell added to queue or value changed in queue: ", added_to_queue_count)                

    return true
end