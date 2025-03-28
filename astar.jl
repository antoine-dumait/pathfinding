include("readMap.jl")
include("utils.jl")

using DataStructures
using Printf
using Plots

#fix neighbor method choise, fix implementation to always return a movemment cost, use with other algorithm
function randomPathAStar(filePath::String, heuristicNumber::Int=1)
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
    while !algoAstarAux((rand(1:width), rand(1:height)), (rand(1:width), rand(1:height)), myCells)
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

function algoAstar(path::String, start::Tuple{Int64,Int64}=(0,0), goal::Tuple{Int64,Int64}=(0,0))
    cells = fileToMatrixGraph(path, 5, 8)
    height, width = size(cells)
    while !checkIfPosWalkable(start, cells)
        start = (rand(1:width), rand(1:height))
    end
    while !checkIfPosWalkable(goal, cells)
        goal = (rand(1:width), rand(1:height))
    end
    @printf("--------------------A*------------------------\n")
    @printf("Finding path from %s to %s\n", start, goal)
    if !checkIfPathPossible(start, goal, cells)
        return false
    end
    @printf("Start and Goal walkable, continuing path finding\n")
    timeStamp = time()
    cheminTrouveBool, came_from, looked_at_count, added_to_queue_count =  algoAstarAux(start::Tuple{Int64,Int64}, goal::Tuple{Int64,Int64}, cells)
    doneTime = time()-timeStamp
    caseCount, path_size = showPathPlotsMatrix!(cells, came_from, start, goal)
    @printf("Taille du chemin: %d\n", path_size)
    println("Cell looked at: ", looked_at_count)
    println("Cell added to queue or value changed in queue: ", added_to_queue_count)                
    @printf("Temps utilisé: %f\n", doneTime)
end

function algoAstarTesting(path::String, start::Tuple{Int64,Int64}=(0,0), goal::Tuple{Int64,Int64}=(0,0))
    cells = fileToMatrixGraph(path, 5, 8)
    height, width = size(cells)
    while !checkIfPosWalkable(start, cells)
        start = (rand(1:width), rand(1:height))
    end
    while !checkIfPosWalkable(goal, cells)
        goal = (rand(1:width), rand(1:height))
    end
    @printf("--------------------A*------------------------\n")
    @printf("Finding path from %s to %s\n", start, goal)
    if !checkIfPathPossible(start, goal, cells)
        return false
    end
    @printf("Start and Goal walkable, continuing path finding\n")
    timeStamp = time()
    cheminTrouveBool, came_from, looked_at_count, added_to_queue_count =  algoAstarAux(start::Tuple{Int64,Int64}, goal::Tuple{Int64,Int64}, cells)
    doneTime = time()-timeStamp
    caseCount, path_size = getPathLengthMatrix(cells, came_from, start, goal)
    @printf("Taille du chemin: %d\n", path_size)
    println("Cell looked at: ", looked_at_count)
    println("Cell added to queue or value changed in queue: ", added_to_queue_count)                
    @printf("Temps utilisé: %f\n", doneTime)
end

function algoAstarAux(start::Tuple{Int64,Int64}, goal::Tuple{Int64,Int64}, cells::Matrix{Int64})#add heurisitc choice    
    inf = typemax(Int64) 
    height, width = size(cells)
    
    distance = fill(inf, height, width) 
    distance[start[2], start[1]] = 0
    
    came_from = Array{Union{Nothing,Tuple{Int64,Int64}}, 2}(nothing, height, width) #array more memory but faster

    isDone = falses(height, width)
    
    queue = BinaryMinHeap{Tuple{Int64,Int64,Int64}}() #BinaryMinHeap faster than PriorityQueue but more memory
    push!(queue, (0, start[1], start[2]))
    
    looked_at_count = 0
    added_to_queue_count = 0
    goalFound = false

    while !isempty(queue)
        looked_at_count+=1
        _, x, y = pop!(queue)
        isDone[y, x] = true
        if (x, y) == goal #voir si plus opti dans le mettre direct dans la boucle voisin
            goalFound =  true
            break
        end
        for (nx, ny) in getNeighbors((x, y), cells)
            if !isDone[ny, nx] && cells[ny, nx] != -1   #verifie si accesible et pas deja fait
                new_dist = distance[y, x] + cells[ny, nx]
                if new_dist < distance[ny, nx]
                    added_to_queue_count+=1
                    distance[ny, nx] = new_dist
                    push!(queue, (new_dist + heuristicManathan((nx,ny), goal), nx, ny))
                    came_from[ny, nx] = (x, y)
                end
            end
        end
    end
    if goalFound
        return (true, came_from, looked_at_count, added_to_queue_count)
    else
        return (false, came_from, looked_at_count, added_to_queue_count)
    end 
end

function algoAstarAux2(start::Tuple{Int64,Int64}, goal::Tuple{Int64,Int64}, cells::Matrix{Int64})#add heurisitc choice    
    inf = typemax(Int64) 
    height, width = size(cells)
    
    distance = fill(inf, height, width) 
    distance[start[2], start[1]] = 0
    
    came_from = Array{Tuple{Int64,Int64}, 2}(undef, height, width) #array more memory but faster
    
    isDone = falses(height, width)
    
    queue = BinaryMinHeap{Tuple{Int64,Int64,Int64}}() #BinaryMinHeap faster than PriorityQueue but more memory
    # queue[(start[1], start[2])]= 0  # distance, y, x
    push!(queue, (0, start[1], start[2]))
    
    
    looked_at_count = 0
    added_to_queue_count = 0
    goalFound = false
    while !isempty(queue)
        looked_at_count+=1
        _, x, y = pop!(queue)
        # x, y = dequeue!(queue)
        isDone[y, x] = true
        for (nx, ny) in getNeighbors((x, y), cells)
            if !isDone[ny, nx] && cells[ny, nx] != -1   #verifie si accesible et pas deja fait
                new_dist = distance[y, x] + cells[ny, nx]
                if new_dist < distance[ny, nx]
                    added_to_queue_count+=1
                    distance[ny, nx] = new_dist
                    came_from[ny, nx] = (x, y)
                    if (nx, ny) == goal #voir si plus opti dans le mettre direct dans la boucle voisin
                        return (true, came_from, looked_at_count, added_to_queue_count)                    
                    end
                    push!(queue, (new_dist + heuristicManathan((nx,ny), goal), nx, ny))
                    # queue[(nx,ny)] = new_dist + heuristicManathan((nx,ny), goal)
                end
            end
        end
    end
    return (false, Array{Tuple{Int,Int}}(), looked_at_count, added_to_queue_count)
end