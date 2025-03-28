include("readMap.jl")
include("utils.jl")
include("bfs.jl")
include("djikstra.jl")
include("astar.jl")
include("glouton.jl")

using BenchmarkTools

function testAlgorithms(path::String, start::Tuple{Int64,Int64}, goal::Tuple{Int64,Int64}) #add random possiblites
    algoBFS(path, start, goal)
    algoDjikstra(path, start, goal)
    algoAstar(path, start, goal)
    algoGlouton(path, start, goal)
end

function benchAlgo(algo::Function, start::Tuple{Int64,Int64}, goal::Tuple{Int64,Int64}, movement_cost::Array{Int64, 2})
    @printf("Benchmarking algorithm: %s\n", String(Symbol(algo)))
    @printf("Finding path from %s to %s\n", start, goal)
    if !checkIfPathPossible(start, goal, movement_cost)
        return false
    end
    # timeStamp = time()
    @benchmark cheminTrouveBool, came_from, looked_at_count, added_to_queue_count =  $algo($start, $goal, $movement_cost)
    # doneTime = time()-timeStamp
    # caseCount, path_size = showPathPlots!(movement_cost, came_from, start, goal)
    # @printf("Taille du chemin: %d\n", path_size)
    # println("Cell looked at: ", looked_at_count)
    # println("Cell added to queue or value changed in queue: ", added_to_queue_count)                
    # @printf("Temps utilisé: %f\n", doneTime)
end
#ne fonctionne pas, ne prend en compte qu'un seul des benchmarks
# function compareAlgo(algo1::Function, algo2::Function, start::Tuple{Int64,Int64}, goal::Tuple{Int64,Int64}, movement_cost::Array{Int64, 2})
#     @printf("Comparing algorithm %s and %s\n", String(Symbol(algo1)), String(Symbol(algo2)))
#     @printf("Path from %s to %s\n", start, goal)
#     if !checkIfPathPossible(start, goal, movement_cost)
#         return false
#     end
#     @benchmark cheminTrouveBool, came_from, looked_at_count, added_to_queue_count =  $algo1($start, $goal, $movement_cost)
#     @benchmark cheminTrouveBool, came_from, looked_at_count, added_to_queue_count =  $algo2($start, $goal, $movement_cost)
    
# end

function testRandomMap(mapFolderPath::String, W::Int=1)
    for filePath in readdir(mapFolderPath)
        cheminTrouveBool, path_size, caseCount, looked_at_count, added_to_queue_count, doneTime = algoWAstarTesting(mapFolderPath * "/" * filePath, W)
        @printf("Trouvé: %5s | Taille chemin: %5d | Case vérifié: %5d | Temps: %f | Poids: %2d\n", cheminTrouveBool, path_size, added_to_queue_count, doneTime, W)
    end
end

function uniq_by(collection, f)
    seen = Set()
    result = []
    for item in collection
        key = f(item)
        if !(key in seen)
            push!(result, item)
            push!(seen, key)
        end
    end
    return result
end

function testRandomMapDicho(mapFolderPath::String)
    open("resultWeight.txt", "w") do file
        for filePath in readdir(mapFolderPath)
            results = algoWAstarDichoWeight(mapFolderPath * "/" * filePath)
            mapName = split(filePath,".")[1]
            write(file,mapName*"\n")
            astar_res = popfirst!(results)
            dist_opti = astar_res[2]
            looked_at_count_init = astar_res[4]
            sort!(results, by=x-> 
                begin
                    (cheminTrouveBool, path_size, caseCount, looked_at_count, added_to_queue_count, doneTime, weight) = x
                        return (looked_at_count,weight)
                end
            )          
            results = uniq_by(results, x->x[4])
            results = [results[1:min(5, size(results,1))]; astar_res]
            for res in results
                cheminTrouveBool, path_size, caseCount, looked_at_count, added_to_queue_count, doneTime, weight = res
                if cheminTrouveBool
                    opt = round((dist_opti/path_size)*100)
                    case_moins = round((looked_at_count_init/looked_at_count)*100)
                    s = @sprintf("Poids: %5d | Taille chemin: %5d | Optimalité: %3d%% | Case vérifié: %5d | Accélération: %3d%%\n", weight, path_size, opt, looked_at_count,  case_moins)
                    write(file, s)
                end
            end
        end
    end
end


mapRepetition = 10
function resumeMapDicho(mapFolderPath::String) #fait vite fait
    timeStamp = time()
    chemin_calcul = 0

    bests = []      
    bests90 = []
    bests95 = []
    bests100 = []

    # Tableaux pour les gains (pourcentage de réduction de cases vérifiées)
    gain_any_arr = Float64[]
    gain90_arr   = Float64[]
    gain95_arr   = Float64[]
    gain100_arr  = Float64[]

    for filePath in readdir(mapFolderPath)
        for i in 1:mapRepetition
            results = algoWAstarDichoWeight(mapFolderPath * "/" * filePath) # utilise exp pour choisir
            chemin_calcul += size(results, 1)
            maxWeight = results[end][7]
            mapName = split(filePath, ".")[1]
            
            # Récupère la solution A* de référence (premier résultat)
            astar_res = popfirst!(results)
            dist_opti = astar_res[2]
            looked_at_count_init = astar_res[4]

            # On trie les résultats pour récupérer la "meilleure" solution selon nos critères
            sort!(results, by = x -> begin
                (cheminTrouveBool, path_size, caseCount, looked_at_count, added_to_queue_count, doneTime, weight) = x
                return (looked_at_count, weight)
            end)
            best = results[1]

            # Calcul du gain pour "best" (tous niveaux)
            gain_any = (looked_at_count_init / best[4]) * 100
            push!(gain_any_arr, gain_any)
            push!(bests, best[7])  # On conserve aussi le poids, si besoin

            # Filtrage pour obtenir best90 (optimalité > 90%)
            filter!(x -> begin
                (cheminTrouveBool, path_size, caseCount, looked_at_count, added_to_queue_count, doneTime, weight) = x
                return dist_opti / path_size > 0.90
            end, results)
            if size(results, 1) != 0
                best90 = results[1]
                gain90 = (looked_at_count_init / best90[4]) * 100
                push!(gain90_arr, gain90)
                push!(bests90, best90[7])
            end

            # Filtrage pour obtenir best95 (optimalité > 95%)
            filter!(x -> begin
                (cheminTrouveBool, path_size, caseCount, looked_at_count, added_to_queue_count, doneTime, weight) = x
                return dist_opti / path_size > 0.95
            end, results)
            if size(results, 1) != 0
                best95 = results[1]
                gain95 = (looked_at_count_init / best95[4]) * 100
                push!(gain95_arr, gain95)
                push!(bests95, best95[7])
            end

            # Filtrage pour obtenir best100 (optimalité == 100%)
            filter!(x -> begin
                (cheminTrouveBool, path_size, caseCount, looked_at_count, added_to_queue_count, doneTime, weight) = x
                return dist_opti == path_size
            end, results)
            if size(results, 1) != 0
                best100 = results[1]
                gain100 = (looked_at_count_init / best100[4]) * 100
                push!(gain100_arr, gain100)
                push!(bests100, best100[7])
            end
        end
    end

    # Calcul et affichage des médianes de poids
    if size(bests90, 1) != 0
        medB90 = bests90[floor(Int, size(bests90, 1) / 2) + 1]
        @printf("taille 90: %d\n", size(bests90, 1))
        @printf("median 90: %d\n", medB90)
    end
    if size(bests95, 1) != 0
        medB95 = bests95[floor(Int, size(bests95, 1) / 2) + 1]
        @printf("taille 95: %d\n", size(bests95, 1))
        @printf("median 95: %d\n", medB95)
    end
    if size(bests100, 1) != 0
        medB100 = bests100[floor(Int, size(bests100, 1) / 2) + 1]
        @printf("taille 100: %d\n", size(bests100, 1))
        @printf("median 100: %d\n", medB100)
    end
    if size(bests, 1) != 0
        medB = bests[floor(Int, size(bests, 1) / 2) + 1]
        @printf("taille any: %d\n", size(bests, 1))
        @printf("median any: %d\n", medB)
    end

    # Calcul et affichage des moyennes de pourcentage de gain en nombre de cases vérifiées
    if !isempty(gain_any_arr)
        avg_gain_any = mean(gain_any_arr)
        @printf("Average gain (any): %.2f%%\n", avg_gain_any)
    end
    if !isempty(gain90_arr)
        avg_gain90 = mean(gain90_arr)
        @printf("Average gain (90%%): %.2f%%\n", avg_gain90)
    end
    if !isempty(gain95_arr)
        avg_gain95 = mean(gain95_arr)
        @printf("Average gain (95%%): %.2f%%\n", avg_gain95)
    end
    if !isempty(gain100_arr)
        avg_gain100 = mean(gain100_arr)
        @printf("Average gain (100%%): %.2f%%\n", avg_gain100)
    end

    @printf("time taken: %f\n", time() - timeStamp)
    @printf("nombre de chemin calculé: %d\n", chemin_calcul)
end


