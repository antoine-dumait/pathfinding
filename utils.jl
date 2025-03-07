using DataStructures
using Printf
using Plots

#return (x,y, movement cost)
SQRT2 = sqrt(2)
function getNeighborsDiagonal(pos::Tuple{Int64,Int64}, cells::Matrix{Int64})
    neighbors_index= [(0,-1),(1,0),(0,1),(-1,0),  #haut, droite, bas, gauche
                        (1,-1), (-1,-1),(1,1),(-1,1)]#haut-droite, haut-gauche, bas-droite, bas-gauche
    x,y = pos #Int64
    width, height = size(cells)
    neighbors = Vector{Tuple{Int64,Int64,Int64}}()
    for (dx,dy) in neighbors_index
        nx, ny = x+dx, y+dy
        if nx >= 1 && nx <= width && ny >= 1 && ny <= height && cells[ny, nx] != -1
            if abs(nx) + abs(ny) == 2 
                if cells[ny, x] == -1 || cells[y, nx] == -1
                    continue
                end
                push!(neighbors, (nx, ny, sqrt(2)))
            else
                push!(neighbors, (nx, ny, 1))
            end
        end
    end
    return neighbors
end

function getNeighbors(pos::Tuple{Int64,Int64}, cells::Matrix{Int64})
    neighbors_index= [(1,0),(0,1),(0,-1),(-1,0)] #haut, droite, bas, gauche
    x,y = pos #Int64
    width, height = size(cells)
    neighbors::Vector{Tuple{Int64,Int64}} = Vector{Tuple{Int64,Int64}}()
    for (dx,dy) in neighbors_index
        nx, ny = (x+dx, y+dy)
        if (nx >= 1 && nx <= width && ny >= 1 && ny <= height && cells[ny, nx] != -1)
            push!(neighbors, (nx, ny))
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
    # println("while 1")
    while haskey(path, prev2) && i < 1000
        
        i+=1
        path_length+=1
        prev2 = path[prev2]
    end
    # @printf("PATH -------------------\nEnd: %s\n", to)
    i=0
    # println("while 2")
    while haskey(path, prev) && i< 1000
        i+=1
        # print("la")
        print(cells[prev[2],prev[1]])
        @printf("%d: %s\n", path_length, path[prev])
        path_length-=1
        prev = path[prev]
    end
    # @printf("Start: %s\n", start)
end

function showPathPlots!(cells::Matrix{Int64}, path::Dict{Tuple{Int64,Int64},Tuple{Int64,Int64}}, start::Tuple{Int64,Int64}, goal::Tuple{Int64,Int64})
    println("In showPathPlots!")
    path_size = 1
    prev = goal
    while haskey(path, prev) && path[prev] != start
        # println("pathsize: ", path_size, " cost: ", cells[prev[2], prev[1]])
        path_size += cells[prev[2], prev[1]]
        prev = path[prev]
    end

    for (key, _) in path
        cells[key[2], key[1]] = 2
    end

    prev = goal
    while haskey(path, prev) && path[prev] != start
        cells[prev[2], prev[1]] = 3
        prev = path[prev]
    end

    if haskey(path, prev)  
        cells[prev[2], prev[1]] = 3
    end

    cells[start[2], start[1]] = -3
    cells[goal[2], goal[1]] = -2        

    colors = cgrad([:white, :blue, :grey23, :grey23, :burlywood1, :seagreen4, :red2], [-3, -2, -1,0, 1, 2, 3])

    p = heatmap(cells,c=colors, yaxis=:flip, clim=(minimum(cells), maximum(cells)))
    
    display(p)
    @printf("Taille du chemin: %d\n", path_size)
    println("Done!")
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
    elseif (cells[start[2],start[1]] == -1)
        @printf("Start %s is not walkable. Stopping\n", start)
        return false
    elseif (cells[goal[2],goal[1]] == -1)
        @printf("End %s is not walkable. Stopping\n", goal)
        return false
    end
    return true
end