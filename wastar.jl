include("readMap.jl")
include("utils.jl")

using DataStructures
using Printf
using Plots

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

function algoWAstar(path::String, W::Int, start::Tuple{Int64,Int64}=(0,0), goal::Tuple{Int64,Int64}=(0,0))
    cells = fileToMatrixGraph(path, 5, 8)
    height, width = size(cells)
    while !checkIfPosWalkable(start, cells)
        start = (rand(1:width), rand(1:height))
    end
    while !checkIfPosWalkable(goal, cells)
        goal = (rand(1:width), rand(1:height))
    end
    @printf("--------------------WA*------------------------\n")
    @printf("Finding path from %s to %s\n", start, goal)
    if !checkIfPathPossible(start, goal, cells)
        return false
    end
    @printf("Start and Goal walkable, continuing path finding\n")
    timeStamp = time()
    cheminTrouveBool, came_from, looked_at_count, added_to_queue_count =  algoWAstarAux(W,start, goal, cells)
    doneTime = time()-timeStamp
    caseCount, path_size = showPathPlotsMatrix!(cells, came_from, start, goal)
    @printf("Weight: %d\n", W)
    @printf("Taille du chemin: %d\n", path_size)
    println("Cell looked at: ", looked_at_count)
    println("Cell added to queue or value changed in queue: ", added_to_queue_count)                
    @printf("Temps utilisé: %f\n", doneTime)
end

function algoWAstarTesting(path::String, W::Int, start::Tuple{Int64,Int64}=(0,0), goal::Tuple{Int64,Int64}=(0,0))
    cells = fileToMatrixGraph(path, 5, 8)
    height, width = size(cells)
    while !checkIfPosWalkable(start, cells)
        start = (rand(1:width), rand(1:height))
    end
    while !checkIfPosWalkable(goal, cells)
        goal = (rand(1:width), rand(1:height))
    end
    if !checkIfPathPossible(start, goal, cells)
        return (false, 0, 0, 0, 0)
    end
    timeStamp = time()
    cheminTrouveBool, came_from, looked_at_count, added_to_queue_count =  algoWAstarAux(W, start::Tuple{Int64,Int64}, goal::Tuple{Int64,Int64}, cells)
    doneTime = time()-timeStamp
    caseCount, path_size = getPathLengthMatrix(cells, came_from, start, goal)
    return (cheminTrouveBool, path_size, caseCount, looked_at_count, added_to_queue_count, doneTime)
end


#augmente la valeur de W jusqu'a ce que celle-ci ne diminue plus le nombres de cases évalués
function algoWAstarDichoWeight(path::String, start::Tuple{Int64,Int64}=(0,0), goal::Tuple{Int64,Int64}=(0,0))
    cells = fileToMatrixGraph(path, 5, 8)
    height, width = size(cells)
    while !checkIfPosWalkable(start, cells)
        start = (rand(1:width), rand(1:height))
    end
    while !checkIfPosWalkable(goal, cells)
        goal = (rand(1:width), rand(1:height))
    end
    if !checkIfPathPossible(start, goal, cells)
        return (false, 0, 0, 0, 0)
    end

    results = []
    alpha = 0.5
    fCalcW = x->exp(x*alpha)
    x=0
    maxX = 10
    W = 1
    last_count = Core.typemax_Int       
    no_improvement_count = 0
    max_no_improve = 3 #arrete de calculer de nouveau W si 3 fois de suite celui ci ne sameliore pas
    while no_improvement_count < max_no_improve || x < maxX
        timeStamp = time()
        cheminTrouveBool, came_from, looked_at_count, added_to_queue_count =  algoWAstarAux(W, start, goal, cells)
        doneTime = time()-timeStamp
        caseCount, path_size = getPathLengthMatrix(cells, came_from, start, goal)
        push!(results , (cheminTrouveBool, path_size, caseCount, looked_at_count, added_to_queue_count, doneTime, W))
        if last_count <= looked_at_count
            no_improvement_count +=1
        else
            last_count = looked_at_count
        end
        x+=1 
        W = round(Int,fCalcW(x))

    end
    return results
end


function algoWAstarAux(W::Int, start::Tuple{Int64,Int64}, goal::Tuple{Int64,Int64}, cells::Matrix{Int64})#add heurisitc choice    
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
                    push!(queue, (new_dist + W*heuristicManathan((nx,ny), goal), nx, ny))
                    came_from[ny, nx] = (x, y)
                end
            end
        end
    end
    # goalFound = isDone[goal[2],goal[1]] == true
    if goalFound
        return (true, came_from, looked_at_count, added_to_queue_count)
    else
        return (false, came_from, looked_at_count, added_to_queue_count)
    end 
end

function algoWAstarDynamicWeight(start::Tuple{Int,Int}, goal::Tuple{Int,Int}, cells::Matrix{Int64})
    inf = typemax(Int64)
    height, width = size(cells)

    distance = fill(inf, height, width)
    distance[start[2], start[1]] = 0

    came_from = Array{Union{Nothing, Tuple{Int,Int}}, 2}(nothing, height, width)

    isDone = falses(height, width)
    queue = BinaryMinHeap{Tuple{Float64, Int, Int}}()  # poids dynamique en float

    push!(queue, (0.0, start[1], start[2]))

    looked_at_count = 0
    added_to_queue_count = 0
    goalFound = false

    max_manhattan = heuristicManathan(start, goal) + 1e-6  # éviter division par zéro

    while !isempty(queue)
        looked_at_count += 1
        _, x, y = pop!(queue)
        isDone[y, x] = true

        if (x, y) == goal
            goalFound = true
            break
        end

        for (nx, ny) in getNeighbors((x, y), cells)
            if !isDone[ny, nx] && cells[ny, nx] != -1
                new_dist = distance[y, x] + cells[ny, nx]

                if new_dist < distance[ny, nx]
                    added_to_queue_count += 1
                    distance[ny, nx] = new_dist

                    
                    manh = heuristicManathan((nx, ny), goal)
                    dynamic_W = manh / max_manhattan  #W calculé en fonction de la distance à l'arrivée
                    priority = new_dist + dynamic_W * manh 
                    push!(queue, (priority, nx, ny))

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
