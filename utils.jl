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
    height, width = size(cells)
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
    path_size = 1
    cell_count_path = 1 #commence 1 car compte pas la fin donc faut la rajouter
    prev = goal
    while haskey(path, prev) && path[prev] != start
        path_size += cells[prev[2], prev[1]]
        cell_count_path += 1
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
    return (cell_count_path, path_size)
end

function showPathPlotsMatrix!(cells::Matrix{Int64}, path_matrix::Array{Union{Nothing,Tuple{Int64,Int64}}, 2}, start::Tuple{Int64,Int64}, goal::Tuple{Int64,Int64})
    path_size = 1
    cell_count_path = 1 #commence 1 car compte pas la fin donc faut la rajouter
    height, width = size(cells)
    checked_cell_color_value = 2
    path_color_value = 3
    color_array = Array{Int64, 2}(undef, height, width) 
    fill!(color_array,1)
    for x in 1:width, y in 1:height
        if !isnothing(path_matrix[y,x])          #TODO 26/03 FINIR CA ET VOIR OBSIDIAN POUR RECAP
            color_array[y,x] = checked_cell_color_value
        else
            color_array[y,x] = cells[y,x]
        end
    end

    prev = goal
    while !isnothing(path_matrix[prev[2], prev[1]]) && prev != start
        path_size += cells[prev[2], prev[1]]
        cell_count_path += 1        
        color_array[prev[2], prev[1]] = path_color_value
        prev = path_matrix[prev[2], prev[1]]
    end

    # if haskey(path, prev)  
    #     cells[prev[2], prev[1]] = 3
    # end
    start_color_value = -3
    goal_color_value = -2
    color_array[start[2], start[1]] = -3
    color_array[goal[2], goal[1]] = -2        

    colors = cgrad([:white, :blue, :grey23, :burlywood1, :burlywood1, :seagreen4, :red2], [-3, -2, -1,0, 1, 2, 3])
    p = heatmap(color_array,c=colors, yaxis=:flip)

    display(p)
    return (cell_count_path, path_size)
end

function getPathLengthMatrix(cells::Matrix{Int64}, path_matrix::Array{Union{Nothing,Tuple{Int64,Int64}}, 2}, start::Tuple{Int64,Int64}, goal::Tuple{Int64,Int64})
    path_size = 1
    cell_count_path = 1 #commence 1 car compte pas la fin donc faut la rajouter
    prev = goal
    while !isnothing(path_matrix[prev[2], prev[1]]) && prev != start
        path_size += cells[prev[2], prev[1]]
        cell_count_path += 1        
        prev = path_matrix[prev[2], prev[1]]
    end
    return (cell_count_path, path_size)
end

function checkIfPosWalkable(pos::Tuple{Int64,Int64}, cells::Matrix{Int64})
    height, width = size(cells)
    return !(pos[1] < 1 || pos[1] > width || pos[2] < 1 || pos[2] > height || cells[pos[2],pos[1]] == -1)
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

function heuristicManathan(start::Tuple{Int64,Int64}, goal::Tuple{Int64,Int64})
    return abs(start[1] - goal[1]) + abs(start[2] - goal[2])
end

function heuristicEuclid(start::Tuple{Int64,Int64}, goal::Tuple{Int64,Int64}) #heurristique eculidienne optimisite car utilise cout de mouvment de 1
    return Int64(floor(sqrt((start[1] - goal[1])^2 + (start[2] - goal[2])^2)))
end