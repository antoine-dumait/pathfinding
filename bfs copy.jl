include("readMap.jl")
include("utils.jl")
using DataStructures
using Printf
using Plots

myCells::Matrix{Int64} = fileToMatrixBFS("maps/256.map")
height, width = size(myCells)

function getPath(start::Tuple{Int64,Int64}, goal::Tuple{Int64,Int64}, cells::Matrix{Int64})
    @printf("Finding path from %s to %s\n", start, goal)

    if !checkIfPathPossible(start, goal, cells)
        return false
    end

    toCheck = Queue{Tuple{Int64, Int64}}()
    enqueue!(toCheck, start)
    came_from = Dict{Tuple{Int64, Int64}, Tuple{Int64, Int64}}()
    came_from[start] = (0, 0) #montre que cest point de depart
    visited = Set{Tuple{Int64, Int64}}() #matrice de faux ?

    while !isempty(toCheck)
        current_pos = dequeue!(toCheck)
        push!(visited, current_pos)

        if current_pos == goal
            showPath(came_from, start, goal, cells)
            showPathPlots!(cells, came_from, start, goal)
            return true
        end

        for next in getNeighbors(current_pos, cells)
            if !haskey(came_from, next)
                enqueue!(toCheck, next)
                came_from[next] = current_pos
            end
        end
    end

    @printf("No path found between %s and %s\n", start, goal)
    return false
end

# Exécuter jusqu'à trouver un chemin valide
while !getPath((rand(1:width), rand(1:height)), (rand(1:width), rand(1:height)), myCells)
end
