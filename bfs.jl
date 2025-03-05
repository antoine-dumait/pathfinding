include("readMap.jl")
include("utils.jl")
using DataStructures
using Printf
using Plots
myCells::Matrix{Int64} = fileToMatrixBFS("maps/Sydney_1_256.map")
height, width = size(myCells)


function getPath(start::Tuple{Int64,Int64}, goal::Tuple{Int64,Int64}, cells::Matrix{Int64})
    @printf("getPath from %s to %s\n", start, goal)
    height, width = size(cells)
    if !(checkIfPathPossible(start,goal,cells))
        return false
    end
    @printf("Start and Goal walkable, continuing path finding\n")
    @show(height, width)
    check_count = 0
    toCheck = Queue{Tuple{Int64,Int64}}()
    enqueue!(toCheck, start)
    came_from = Dict{Tuple{Int64,Int64},Tuple{Int64,Int64}}()
    came_from[start] = (0,0)
    visited = []
    
    while !isempty(toCheck)
        check_count+=1
        current_pos::Tuple{Int64,Int64} = dequeue!(toCheck)
        push!(visited, current_pos)
        if (check_count % 100 == 0)
            # @printf("Check count: %s\n", check_count)
        end
        # @printf("count %d; currently checking %s\n", check_count, current_pos)
        if current_pos == goal
            showPath(came_from, start, goal, cells)
            showPathPlots!(cells, came_from, start, goal)
            return true
        end
        for next in getNeighbors(current_pos, cells)
            if !(next in keys(came_from))
                enqueue!(toCheck, next)
                came_from[next] = current_pos
            end
        end
        # break
    end
    @printf("No path found between %s and %s in map", start, goal)
    return false
end

while !getPath((rand(1:width),rand(1:height)) ,(rand(1:width),rand(1:height)),myCells)
end
# print(myCells)
# heatmap(myCells)