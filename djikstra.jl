include("readMap.jl")
include("utils.jl")

using DataStructures
using Printf
using Plots

myCells::Matrix{Int64} = fileToMatrixGraph("maps/me.map", 2)
height, width = size(myCells)
#valeur choisit pour infini Int64
inf = 999999

function showDist(distances::Matrix{Int64})
    p = heatmap(transpose(distances), yaxis=:flip)
    display(p)
end

# function getMinDist(distances::Matrix{Int64}, done::Matrix{Bool})
#     min_dist = distances[1,1]
#     index = (1,1)
#     height, width = size(distances)
#     for y in 1:height for x in 1:width
#         if distances[y,x] < min_dist && done[y,x] == false
#         end
#     end end
# end
function getPath(start::Tuple{Int64,Int64}, goal::Tuple{Int64,Int64}, cells::Matrix{Int64})
    @printf("getPath from %s to %s\n", start, goal)
    height, width = size(cells)
    if !checkIfPathPossible(start,goal,cells)
        return false
    end
    @printf("Start and Goal walkable, continuing path finding\n")
    @show(height, width)
    check_count = 0
    toCheck = []
    came_from = Dict{Tuple{Int64,Int64},Tuple{Int64,Int64}}()
    came_from[start] = (0,0)
    distance = Matrix{Int64}(undef, height, width)
    fill!(distance, inf)
    isDone = Matrix{Bool}(undef, height, width)
    for x = 1:width, y = 1:height
        if(cells[x,y] != -1)
            distance[x,y] = 999999
            isDone[x,y] = false
            push!(toCheck, (x,y))
        else 
            distance[x,y] = -1
        end
    end
    distance[start[1],start[2]] = 0
    sort!(toCheck, by=x->distance[x[1],x[2]])
    endFound = false
    n=0
    while !isempty(toCheck) 
        n+=1
        if n%10==0
            @show(n)
        end
        actuel = popfirst!(toCheck)
        isDone[actuel[1],actuel[2]] = true
        if actuel == goal
            endFound = true
        end
        for cell in getNeighbors(actuel, cells)
            dist = min(distance[cell[1],cell[2]], cells[cell[1], cell[2]] + distance[cell[1],cell[2]])
            if dist < distance[cell[1],cell[2]]
                distance[cell[1],cell[2]] = dist
                came_from[cell] = actuel
            end
        end
        sort!(toCheck, by=x->distance[x[1],x[2]])
    end
    distMatrix::Matrix{Int64} = Matrix{Int64}(undef, height, width)
    @show(distance)
    showDist(distMatrix)
    # showPathPlots!(cells, came_from, start, goal)
    return endFound

end

getPath((rand(1:width),rand(1:height)) ,(rand(1:width),rand(1:height)),myCells)
# while !getPath((rand(1:width),rand(1:height)) ,(rand(1:width),rand(1:height)),myCells)
# end



