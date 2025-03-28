include("readMap.jl")
include("utils.jl")
include("bfs.jl")
include("djikstra.jl")
include("astar.jl")
include("glouton.jl")

using BenchmarkTools

function testAlgorithms(path::String, start::Tuple{Int64, Int64}, goal::Tuple{Int64, Int64})
    algoBFS(path, start, goal)
    algoDjikstra(path, start, goal)
    algoAstar(path, start, goal)
    algoGlouton(path, start, goal)
    algoWAstar(2, path, start, goal)
end

function benchAlgo(algo::Function, start::Tuple{Int64, Int64}, goal::Tuple{Int64, Int64}, movement_cost::Array{Int64, 2})
    @printf("Benchmarking algorithm: %s\n", String(Symbol(algo)))
    @printf("Finding path from %s to %s\n", start, goal)
    if !checkIfPathPossible(start, goal, movement_cost)
        return false
    end
    @benchmark pathFoundBool, came_from, looked_at_count, added_to_queue_count = $algo($start, $goal, $movement_cost)
end

function testRandomMap(mapFolderPath::String, W::Int = 1)
    for filePath in readdir(mapFolderPath)
        pathFoundBool, path_size, caseCount, looked_at_count, added_to_queue_count, doneTime = algoWAstarTesting(mapFolderPath * "/" * filePath, W)
        @printf("Found: %5s | Path size: %5d | Verified cells: %5d | Time: %f | Weight: %2d\n", pathFoundBool, path_size, added_to_queue_count, doneTime, W)
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

function testDifferentWeightDichoFolder(mapFolderPath::String)
    open("resultWeight.txt", "w") do file
        for filePath in readdir(mapFolderPath)
            results = algoWAstarDichoWeight(mapFolderPath * "/" * filePath)
            mapName = split(filePath, ".")[1]
            write(file, mapName * "\n")

            astar_res = popfirst!(results)
            dist_opti = astar_res[2]
            looked_at_count_init = astar_res[4]

            sort!(results, by = x -> begin
                (_, _, _, looked_at_count, _, _, weight) = x
                return (looked_at_count, weight)
            end)

            results = uniq_by(results, x -> x[4])
            results = [results; astar_res]

            for res in results
                pathFoundBool, path_size, caseCount, looked_at_count, added_to_queue_count, doneTime, weight = res
                if pathFoundBool
                    opt = round((dist_opti / path_size) * 100)
                    case_less = round((looked_at_count_init / looked_at_count) * 100)
                    s = @sprintf("Weight: %5d | Path size: %5d | Optimality: %3d%% | Verified cells: %5d | Acceleration: %3d%%\n", weight, path_size, opt, looked_at_count, case_less)
                    write(file, s)
                end
            end
        end
    end
end

mapRepetition = 10

function resumeMapDicho(mapFolderPath::String)
    timeStamp = time()
    path_calculated = 0

    bests     = []
    bests90   = []
    bests95   = []
    bests100  = []

    gain_any_arr  = Float64[]
    gain90_arr    = Float64[]
    gain95_arr    = Float64[]
    gain100_arr   = Float64[]

    for filePath in readdir(mapFolderPath)
        for _ in 1:mapRepetition
            results = algoWAstarDichoWeight(mapFolderPath * "/" * filePath)
            path_calculated += size(results, 1)

            astar_res = popfirst!(results)
            dist_opti = astar_res[2]
            looked_at_count_init = astar_res[4]

            sort!(results, by = x -> begin
                (_, _, _, looked_at_count, _, _, weight) = x
                return (looked_at_count, weight)
            end)

            best = results[1]
            gain_any = (looked_at_count_init / best[4]) * 100
            push!(gain_any_arr, gain_any)
            push!(bests, best[7])

            filter!(x -> dist_opti / x[2] > 0.90, results)
            if !isempty(results)
                best90 = results[1]
                gain90 = (looked_at_count_init / best90[4]) * 100
                push!(gain90_arr, gain90)
                push!(bests90, best90[7])
            end

            filter!(x -> dist_opti / x[2] > 0.95, results)
            if !isempty(results)
                best95 = results[1]
                gain95 = (looked_at_count_init / best95[4]) * 100
                push!(gain95_arr, gain95)
                push!(bests95, best95[7])
            end

            filter!(x -> dist_opti == x[2], results)
            if !isempty(results)
                best100 = results[1]
                gain100 = (looked_at_count_init / best100[4]) * 100
                push!(gain100_arr, gain100)
                push!(bests100, best100[7])
            end
        end
    end

    if !isempty(bests90)
        medB90 = bests90[floor(Int, length(bests90) / 2) + 1]
        @printf("size 90: %d\n", length(bests90))
        @printf("median 90: %d\n", medB90)
    end

    if !isempty(bests95)
        medB95 = bests95[floor(Int, length(bests95) / 2) + 1]
        @printf("size 95: %d\n", length(bests95))
        @printf("median 95: %d\n", medB95)
    end

    if !isempty(bests100)
        medB100 = bests100[floor(Int, length(bests100) / 2) + 1]
        @printf("size 100: %d\n", length(bests100))
        @printf("median 100: %d\n", medB100)
    end

    if !isempty(bests)
        medB = bests[floor(Int, length(bests) / 2) + 1]
        @printf("size any: %d\n", length(bests))
        @printf("median any: %d\n", medB)
    end

    if !isempty(gain_any_arr)
        @printf("Average gain (any): %.2f%%\n", mean(gain_any_arr))
    end
    if !isempty(gain90_arr)
        @printf("Average gain (90%%): %.2f%%\n", mean(gain90_arr))
    end
    if !isempty(gain95_arr)
        @printf("Average gain (95%%): %.2f%%\n", mean(gain95_arr))
    end
    if !isempty(gain100_arr)
        @printf("Average gain (100%%): %.2f%%\n", mean(gain100_arr))
    end

    @printf("Time taken: %f\n", time() - timeStamp)
    @printf("Number of paths calculated: %d\n", path_calculated)
end
