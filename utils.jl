using DataStructures
using Printf
using Plots

neighbors_index= [(0,1), (1,0),(0,-1),(-1,0)] #haut, droite, bas, gauche
function getNeighbors(pos::Tuple{Int64,Int64}, cells::Matrix{Int64})
    x,y = pos #Int64
    maxX, maxY = size(cells)
    neighbors::Vector{Tuple{Int64,Int64}} = Vector{Tuple{Int64,Int64}}()
    for (xIncr,yIncr) in neighbors_index
        xNew, yNew = (x+xIncr, y+yIncr)
        if (xNew >= 1 && xNew <= maxX && yNew >= 1 && yNew <= maxY && cells[xNew, yNew] != -1)
            push!(neighbors, (xNew, yNew))
        end
    end
    return neighbors
end

# make functions that gets the path and calculate length same time, use recursive
# function getPrevPath(path::Dict{Tuple{Int64,Int64},Tuple{Int64,Int64}}, curr::Tuple{Int64,Int64}, index::Int64)
#     # return 
# end
function showPath(path::Dict{Tuple{Int64,Int64},Tuple{Int64,Int64}}, start::Tuple{Int64,Int64}, to::Tuple{Int64,Int64}, cells::Matrix{Int64})
    prev = to
    prev2 = to
    path_length = 0
    i=0
    println("while 1")
    while path[prev2] != start && i < 1000
        
        i+=1
        path_length+=1
        prev2 = path[prev2]
    end
    @printf("PATH -------------------\nEnd: %s\n", to)
    i=0
    println("while 2")
    while path[prev] != start && i< 1000
        i+=1
        print("la")
        print(cells[prev[1],prev[2]])
        @printf("%d: %s\n", path_length, path[prev])
        path_length-=1
        prev = path[prev]
    end
    @printf("Start: %s\n", start)
end

function showPathPlots!(cells::Matrix{Int64}, path::Dict{Tuple{Int64,Int64},Tuple{Int64,Int64}}, start::Tuple{Int64,Int64}, to::Tuple{Int64,Int64})
    println("in showPath ")
    for (key, value) in path 
        cells[key[1],key[2]] = 3
    end
    prev = to
    # println("while 1")
    i=0
    while path[prev] != start && i < 100
        # println(prev, path[prev])
        i+=1
        cells[prev[1],prev[2]] = 2
        prev = path[prev]
    end
    cells[prev[1],prev[2]] = 2
    cells[start[1],start[2]] = -3
    cells[to[1],to[2]] = -2


    # cells[100,100] = 2
    p = heatmap(cells, yaxis=:flip)
    # p = heatmap(cells, c=cgrad([:green, :blue, :black, :yellow, :red], [-3.0, -2.0, -1.0, 0.25, 0.5, 0.75, 1.0]), clim=(0, 100))

    # p = heatmap(randn(10,10))
    display(p)
    print("Done !")
end

function checkIfPathPossible(start::Tuple{Int64,Int64}, goal::Tuple{Int64,Int64}, cells::Matrix{Int64})
    #check bounds before walkable !
    height, width = size(cells)
    if (start[1] < 1 || start[1] > width || start[2] < 1 || start[2] > height)
        @printf("Start %s is out of bounds. Stopping\n", start)
        return false
    elseif (goal[1] < 1 || goal[1] > width || goal[2] < 1 || goal[2] > height)
        @printf("End %s is out of bounds. Stopping\n", goal)
        return false
    elseif (cells[start[1],start[2]] == -1)
        @printf("Start %s is not walkable. Stopping\n", start)
        return false
    elseif (cells[goal[1],goal[2]] == -1)
        @printf("End %s is not walkable. Stopping\n", goal)
        return false
    end
    return true
end