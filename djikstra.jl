include("readMap.jl")
include("utils.jl")

using DataStructures
using Printf
using Plots

myCells::Matrix{Int64} = fileToMatrixGraph("maps/256.map", 2)
height, width = size(myCells)
inf = typemax(Int64) 

function showDist(distances::Matrix{Int64})
    distances2 = map(x-> x != typemax(Int64) ? x : -1, distances)
    p = heatmap(distances2, yaxis=:flip)
    display(p)
end

function getPath(start::Tuple{Int64,Int64}, goal::Tuple{Int64,Int64}, cells::Matrix{Int64})
    @printf("getPath from %s to %s\n", start, goal)
    height, width = size(cells)
    
    if !checkIfPathPossible(start, goal, cells)
        return false
    end
    
    @printf("Start and Goal walkable, continuing path finding\n")
    check_count = 0
    
    distance = fill(inf, height, width)
    came_from = Dict{Tuple{Int64,Int64},Tuple{Int64,Int64}}()
    isDone = falses(height, width)
    
    heap = BinaryMinHeap{Tuple{Int64,Int64,Int64}}()
    push!(heap, (0, start[1], start[2]))  # distance, x,y 
    
    distance[start[2], start[1]] = 0
    
    while !isempty(heap)
        dist, x, y = pop!(heap)

        isDone[y, x] = true
        
        if (x,y) == goal
            # break
        end
        
        for (nx, ny) in getNeighbors((x, y), cells)
            if !isDone[ny, nx] && cells[ny, nx] != -1  #verifie si accesible
                new_dist = dist + cells[ny, nx]
                if new_dist < distance[ny, nx]
                    distance[ny, nx] = new_dist
                    push!(heap, (new_dist, nx, ny))
                    came_from[(nx, ny)] = (x, y)
                end
            end
        end
    end
    
    if distance[goal...] == inf
        return false
    end
    showDist(distance)
    return true
end

#test alea
getPath((rand(1:height), rand(1:width)), (rand(1:height), rand(1:width)), myCells)
