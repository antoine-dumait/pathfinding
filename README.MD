# Pathfinding Algorithms Comparison
## Overview
This project implements and compares several pathfinding algorithms in Julia, including:

* Breadth-First Search (BFS)
* Dijkstra's Algorithm
* A* Search
* Greedy Best-First Search
* Weighted A* Search (WA*)

## Maps
Map format used in this project: https://movingai.com/benchmarks/grids.html

### Terrain Representation

* .: Walkable terrain (cost: 1)
* S: Sand (cost: 5)
* W: Water (cost: 8)
* @, O, T: Obstacles (non-walkable)

## Usage
To get the path with any algo and display a visual with the path length and the number of evaluated cells:

``` 
    algoBFS(path, start, goal)
    algoDjikstra(path, start, goal)
    algoAstar(path, start, goal)
    algoGlouton(path, start, goal)
    algoWAstar(weight, path, start, goal)
```
![Example of an output](/images/output_example.png "Example of an output.")


To test all algorithms except WA* on one map use:

``` 
testAlgorithms(path::String, start::Tuple{Int64,Int64}, goal::Tuple{Int64,Int64})
```

To automatically try different weights on WA* use:

``` 
algoWAstarDichoWeight(path::String, start::Tuple{Int64,Int64}=(0,0), goal::Tuple{Int64,Int64}=(0,0))
```

To see how different weight gives different result on different map use:
``` 
testDifferentWeightDichoFolder(mapFolderPath::String)
```
Output sorted by minimum number of verified cells:
``` 
arena
Weight:     2 | Path size:    18 | Optimality: 100% | Verified cells:    18 | Acceleration: 100%
Weight:     1 | Path size:    18 | Optimality: 100% | Verified cells:    18 | Acceleration: 100%
brc501d
Weight:     4 | Path size:   194 | Optimality: 100% | Verified cells:   207 | Acceleration: 1580%
Weight:     3 | Path size:   194 | Optimality: 100% | Verified cells:   208 | Acceleration: 1573%
Weight:    20 | Path size:   268 | Optimality:  72% | Verified cells:   372 | Acceleration: 879%
Weight:    12 | Path size:   268 | Optimality:  72% | Verified cells:   393 | Acceleration: 832%
Weight:     7 | Path size:   242 | Optimality:  80% | Verified cells:   445 | Acceleration: 735%
Weight:     2 | Path size:   230 | Optimality:  84% | Verified cells:   785 | Acceleration: 417%
Weight:     1 | Path size:   194 | Optimality: 100% | Verified cells:  3271 | Acceleration: 100%
...
```

## TODO

* Implement jump point search
* Implement bidirectional ASTAR
* Add multi-threading ???

