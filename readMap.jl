using Printf

# Fonction pour convertir un fichier en matrice (BFS)
function fileToMatrixBFS(path::String)::Matrix{Int64}
    @printf("Reading map in file '%s'\n", path)
    file = open(path, "r")
    lines = readlines(file)
    close(file)  # Fermeture du fichier après lecture

    height = parse(Int64, split(lines[2], " ")[2])
    width = parse(Int64, split(lines[3], " ")[2])
    myMap = Matrix{Int64}(undef, height, width)
    lines = lines[5:end]  # Ignorer les 5 premières lignes de métadonnées

    @printf("Height: %d, Width: %d\n", height, width)

    for (y, line) in enumerate(lines)
        for (x, c) in enumerate(line)
            valu = if c == '.'
                1
            elseif c in ('@', 'T')
                -1
            else
                @error("Unexpected value in map: $c")
                -1
            end
            myMap[y, x] = valu  # ✅ Correction : Inversion des indices
        end
    end
    return myMap  # ✅ La matrice est maintenant correctement orientée
end

# Fonction d'affichage
function printMap(path::String)
    M = fileToMatrixBFS(path)
    println(M)
end

# Fonction pour convertir un fichier en matrice (Graph)
function fileToMatrixGraph(path::String, sandValue::Int64)::Matrix{Int64}
    @printf("Reading map in file '%s'\n", path)
    file = open(path, "r")
    lines = readlines(file)
    close(file)

    height = parse(Int64, split(lines[2], " ")[2])
    width = parse(Int64, split(lines[3], " ")[2])
    myMap = Matrix{Int64}(undef, height, width)
    lines = lines[5:end]

    @printf("Height: %d, Width: %d\n", height, width)

    for (y, line) in enumerate(lines)
        for (x, c) in enumerate(line)
            valu = if c in ('.', 'G')
                1
            elseif c in ('@', 'O', 'T')
                -1
            elseif c == 'S'
                sandValue
            else
                @error("Unexpected value in map: $c")
                -1
            end
            myMap[y, x] = valu  # ✅ Correction : Inversion des indices
        end
    end
    return myMap
end
