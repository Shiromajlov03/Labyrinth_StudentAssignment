using System;
using System.Collections.Generic;
using UnityEngine;

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

        // Priotiry Queue
        var priorityQueue = new List<(Vector2Int pos, float cost)>();
        priorityQueue.Add((start, 0f));

        // Distance and previous node tracking
        var costSoFar = new Dictionary<Vector2Int, float>();
        var previous = new Dictionary<Vector2Int, Vector2Int>();
        costSoFar[start] = 0f;

        Vector2Int[] directions = new Vector2Int[]
        {
            Vector2Int.up,
            Vector2Int.down,
            Vector2Int.left,
            Vector2Int.right,
        };

        while (priorityQueue.Count > 0) 
        {
            // Sort to get the node with the lowest total movement cost
            priorityQueue.Sort((a, b) => a.cost.CompareTo(b.cost));
            var current = priorityQueue[0].pos;
            priorityQueue.RemoveAt(0);

            if(current == goal)
            {
                return ReconstructPath(previous, start, goal);
            }

            foreach(var direction in directions)
            {
                Vector2Int neighbor = current + direction;

                //skip if oob
                if(neighbor.x < 0 || neighbor.x >= mapData.Width ||
                neighbor.y < 0 || neighbor.y >= mapData.Height)
                    continue;

                // Get movement cost
                float moveCost = GetMovementCost(current, neighbor, mapData);
                if (float.IsInfinity(moveCost))
                    continue;

                float newCost = costSoFar[current] + moveCost;

                if (!costSoFar.ContainsKey(neighbor) || newCost < costSoFar[neighbor])
                {
                    costSoFar[neighbor] = newCost;
                    previous[neighbor] = current;
                    priorityQueue.Add((neighbor, newCost));
                }
            }

            // Handle vent movement
            if (mapData.HasVent(current.x, current.y))
            {
                float ventCost = mapData.GetVentCost(current.x, current.y);
                foreach(var otherVent in mapData.GetOtherVentPositions(current))
                {
                    float newCost = costSoFar[current] + ventCost;
                    if (!costSoFar.ContainsKey(otherVent) || newCost < costSoFar[otherVent])
                    {
                        costSoFar[otherVent] = newCost;
                        previous[otherVent] = current;
                        priorityQueue.Add((otherVent, newCost));
                    }
                }
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

    public static float GetMovementCost(Vector2Int from, Vector2Int to, IMapData mapData)
    {
        float baseCost = 1f;

        // Determine wall blocking and cost
        if (from.x == to.x)
        {
            int checkY = Mathf.Max(from.y, to.y);
            float wallCost = mapData.GetHorizontalWallCost(from.x, checkY);
            if (float.IsInfinity(wallCost))
                return float.PositiveInfinity;
            baseCost += wallCost - 1f;
        }
        else
        {
            int checkX = Mathf.Max(from.x, to.x);
            float wallCost = mapData.GetVerticalWallCost(checkX, from.y);
            if (float.IsInfinity(wallCost))
                return float.PositiveInfinity;
            baseCost += wallCost - 1f;
        }

        return baseCost;
    }

    public static bool IsMovementBlocked(Vector2Int from, Vector2Int to, IMapData mapData)
    {
        return float.IsInfinity(GetMovementCost(from, to, mapData));
    }
}