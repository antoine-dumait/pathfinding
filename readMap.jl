using Printf

# Fonction d'affichage
function printMap(path::String)
    M = fileToMatrixBFS(path)
    println(M)
end

#sand = 5, water = 8
# Fonction pour convertir un fichier en matrice (Graph)
function fileToMatrixGraph(path::String, sandValue::Int64, waterValue::Int64, v=false)::Matrix{Int64}
    if v @printf("Reading map in file '%s'\n", path) end
    file = open(path, "r")
    lines = readlines(file)
    close(file)

    height = parse(Int64, split(lines[2], " ")[2])
    width = parse(Int64, split(lines[3], " ")[2])
    myMap = Matrix{Int64}(undef, height, width)
    lines = lines[5:end]

    if v @printf("Height: %d, Width: %d\n", height, width) end

    for (y, line) in enumerate(lines)
        for (x, c) in enumerate(line)
            valu = if c in ('.', 'G')
                1
            elseif c in ('@', 'O', 'T')
                -1
            elseif c == 'S'
                sandValue
            elseif c == 'W'
                waterValue
            else
                @error("Unexpected value in map: $c")
                -1000
            end
            myMap[y, x] = valu 
        end
    end
    return myMap
end
