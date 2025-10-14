using System;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.Assertions.Must;
using UnityEngine.Rendering;

public static class PathfindingAlgorithm
{
    /* <summary>
     TODO: Implement pathfinding algorithm here
     Find the shortest path from start to goal position in the maze.
     
     Dijkstra's Algorithm Steps:
     1. Initialize distances to all nodes as infinity
     2. Set distance to start node as 0
     3. Add start node to priority queue
     4. While priority queue is not empty:
        a. Remove node with minimum distance
        b. If it's the goal, reconstruct path
        c. For each neighbor:
           - Calculate new distance through current node
           - If shorter, update distance and add to queue
     
     MAZE FEATURES TO HANDLE:
     - Basic movement cost: 1.0 between adjacent cells
     - Walls: Some have infinite cost (impassable), others have climbing cost
     - Vents (teleportation): Allow instant travel between distant cells with usage cost
     
     AVAILABLE DATA STRUCTURES:
     - Dictionary<Vector2Int, float> - for tracking distances
     - Dictionary<Vector2Int, Vector2Int> - for tracking previous nodes (path reconstruction)
     - SortedSet<T> or List<T> - for priority queue implementation
     - mapData provides methods to check walls, vents, and boundaries
     
     HINT: Start simple with BFS (ignore wall costs and vents), then extend to weighted Dijkstra
     </summary> */
    public static List<Vector2Int> FindShortestPath(Vector2Int start, Vector2Int goal, IMapData mapData)
    {
        // TODO: Implement your pathfinding algorithm here
        if (start == goal)
            return new List<Vector2Int> { start };

        Queue<Vector2Int> queue = new Queue<Vector2Int>();

        HashSet<Vector2Int> visited = new HashSet<Vector2Int>();

        Dictionary<Vector2Int, Vector2Int> previous = new Dictionary<Vector2Int, Vector2Int>();

        queue.Enqueue(start);
        visited.Add(start);

        Vector2Int[] directions = new Vector2Int[]
        {
            Vector2Int.up,
            Vector2Int.down,
            Vector2Int.left,
            Vector2Int.right,
        };

        while (queue.Count > 0) { 
        
            Vector2Int current = queue.Dequeue();
            if(current == goal)
            {
                return ReconstructPath(previous, start, goal);
            }

            foreach(Vector2Int direction in directions)
            {
                Vector2Int neighbor = current + direction;

                //skip if oob
                if(neighbor.x < 0 || neighbor.x >= mapData.Width ||
                neighbor.y < 0 || neighbor.y >= mapData.Height)
                    continue;

                //skip if already visited
                if (visited.Contains(neighbor)) { 
                continue;
                }
                
                //skip if blocked
                if(IsMovementBlocked(current, neighbor, mapData))
                {
                    continue;
                }

                //valid neighbor add to exploration
                visited.Add(neighbor);
                previous[neighbor] = current;
                queue.Enqueue(neighbor);

            }
        }
        

        Debug.LogWarning("FindShortestPath not implemented yet!");
        return null;
    }

    private static List<Vector2Int> ReconstructPath(Dictionary<Vector2Int, Vector2Int> previous, Vector2Int start, Vector2Int goal)
    {
        List<Vector2Int> path = new List<Vector2Int>();
        Vector2Int current = goal;

        // Reconstruct path backwards from goal to start
        while (current != start)
        {
            path.Add(current);
            current = previous[current];
        }

        path.Add(start);
        path.Reverse(); // Reverse to get start-to-goal order

        return path;
    }

    public static bool IsMovementBlocked(Vector2Int from, Vector2Int to, IMapData mapData)
{
    if (from.x == to.x)
    {
        
        int checkY = Mathf.Max(from.y, to.y);
        return mapData.HasHorizontalWall(from.x, checkY);
    }
    else
    {
        
        int checkX = Mathf.Max(from.x, to.x);
        return mapData.HasVerticalWall(checkX, from.y);
    }
}
}