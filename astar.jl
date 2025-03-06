include("readMap.jl")
include("utils.jl")

using DataStructures
using Printf
using Plots

movementCost::Matrix{Int64} = fileToMatrixGraph("maps/me2.map", 2)
inf = typemax(Int64) 
height, width = size(movementCost)


function heuristicManathan(start::Tuple{Int64,Int64}, goal::Tuple{Int64,Int64})
    return abs(start[1] - goal[1]) + abs(start[2] - goal[2])
end

function heuristicEuclid(start::Tuple{Int64,Int64}, goal::Tuple{Int64,Int64}) #heurristique eculidienne optimisite car utilise cout de mouvment de 1
    return Int64(floor(sqrt((start[1] - goal[1])^2 + (start[2] - goal[2])^2)))
end

function showDist(distances::Matrix{Int64})
    distances2 = map(x-> x != typemax(Int64) ? x : -1, distances)
    p = heatmap(distances2, yaxis=:flip)
    display(p)
end

function getPath(start::Tuple{Int64,Int64}, goal::Tuple{Int64,Int64}, cells::Matrix{Int64})
    @printf("Getting path from %s to %s\n", start, goal)
    
    if !checkIfPathPossible(start, goal, cells)
        return false
    end
    
    @printf("Start and Goal walkable, continuing path finding\n")
    
    height, width = size(cells)
    
    distance = fill(inf, height, width) 
    distance[start[2], start[1]] = 0
    
    came_from = Dict{Tuple{Int64,Int64},Tuple{Int64,Int64}}()
    
    isDone = falses(height, width)
        
    queue = PriorityQueue{Tuple{Int64,Int64},Int64}()
    queue[(start[1], start[2])]= 0  # distance, y, x
    
    
    looked_at_count = 0
    added_to_queue_count = 0
    while !isempty(queue)
        looked_at_count+=1
        x, y = dequeue!(queue)
        isDone[y, x] = true
        for (nx, ny) in getNeighbors((x, y), cells)
            if cells[ny, nx] != -1 && !isDone[ny, nx]   #verifie si accesible et pas deja fait
                if (nx, ny) == goal
                    came_from[(nx, ny)] = (x, y)
                    showPathPlots!(cells, came_from, start, goal)
                    showPath(came_from, start, goal, cells)
                    println("Cell looked at: ", looked_at_count)
                    println("Cell added to queue: ", added_to_queue_count)                
                    return true
                end
                new_dist = distance[y, x] + cells[ny, nx]
                if new_dist < distance[ny, nx]
                    added_to_queue_count+=1
                    distance[ny, nx] = new_dist
                    queue[(nx,ny)] = new_dist + heuristicManathan((nx,ny), goal)
                    println(nx," ",ny," ",new_dist)
                    came_from[(nx, ny)] = (x, y)
                end
            end
        end
    end
    return false
end

#test alea
getPath((rand(1:height), rand(1:width)), (rand(1:height), rand(1:width)), movementCost)
# getPath((1,height), (12,12), movementCost)
