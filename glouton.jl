include("readMap.jl")
include("utils.jl")

using DataStructures
using Printf
using Plots

#fix neighbor method choise, fix implementation to always return a movemment cost, use with other algorithm
function randomPathGlouton(filePath::String, heuristicNumber::Int=1)
    # heuristicChoice::Function = ()->false #
    # neighborsChoice::Function
    if heuristicNumber == 1
        heuristicChoice = heuristicManathan
        neighborsChoice = getNeighbors
    elseif heuristicNumber == 2
        heuristicChoice = heuristicEuclid
        neighborsChoice = getNeighborsDiagonal
    else
        @error("Erreur numéro heurisitique invalide") 
    end
    myCells::Matrix{Int64} = fileToMatrixGraph(filePath, 5, 8)
    height, width = size(myCells)
    while !algoGloutonAux((rand(1:width), rand(1:height)), (rand(1:width), rand(1:height)), myCells)
    end
end

function heuristicManathan(start::Tuple{Int64,Int64}, goal::Tuple{Int64,Int64})
    return abs(start[1] - goal[1]) + abs(start[2] - goal[2])
end

function heuristicEuclid(start::Tuple{Int64,Int64}, goal::Tuple{Int64,Int64}) #heurristique eculidienne optimisite car utilise cout de mouvment de 1
    return Int64(floor(sqrt((start[1] - goal[1])^2 + (start[2] - goal[2])^2)))
end

function showDistAndPath(distances::Matrix{Int64})
    distances2 = map(x-> x != typemax(Int64) ? x : -1, distances)
    p = heatmap(distances2, yaxis=:flip)
    display(p)
end

function algoGlouton(path::String, start::Tuple{Int64,Int64}, goal::Tuple{Int64,Int64})
    # timeStamp = time()
    algoGloutonAux(start::Tuple{Int64,Int64}, goal::Tuple{Int64,Int64}, fileToMatrixGraph(path, 5, 8))
    # @printf("Temps utilisé: %f\n", time()-timeStamp)
end

function algoGloutonAux(start::Tuple{Int64,Int64}, goal::Tuple{Int64,Int64}, cells::Matrix{Int64})
    timeStamp = time()
    
    @printf("--------------------Glouton------------------------\n")
    @printf("Getting path from %s to %s\n", start, goal)
    
    if !checkIfPathPossible(start, goal, cells)
        return false
    end
    
    @printf("Start and Goal walkable, continuing path finding\n")
    
    inf = typemax(Int64) 
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
                    @printf("Temps utilisé: %d\n", timeStamp-time())
                    showPathPlots!(cells, came_from, start, goal)
                    # showPath(came_from, start, goal, cells)
                    println("Cell looked at: ", looked_at_count)
                    println("Cell added to queue or value changed in queue: ", added_to_queue_count)                
                    return true
                end
                new_dist = distance[y, x] + cells[ny, nx]
                if new_dist < distance[ny, nx]
                    added_to_queue_count+=1
                    distance[ny, nx] = new_dist
                    queue[(nx,ny)] = heuristicManathan((nx,ny), goal)
                    came_from[(nx, ny)] = (x, y)
                end
            end
        end
    end
    return false
end