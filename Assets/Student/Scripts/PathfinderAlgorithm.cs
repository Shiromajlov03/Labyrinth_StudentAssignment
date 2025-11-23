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


    /// <summary>
    /// Min-heap implementation for efficient priority queue operations
    /// </summary>
    private class MinHeap
    {
        private readonly List<(Vector2Int pos, float cost)> heap = new List<(Vector2Int, float)>();

        public int Count => heap.Count;

        public void Enqueue(Vector2Int position, float cost)
        {
            heap.Add((position, cost));
            HeapifyUp(heap.Count - 1);
        }

        public (Vector2Int pos, float cost) Dequeue()
        {
            if (heap.Count == 0)
                throw new InvalidOperationException("Heap is empty");

            var result = heap[0];
            heap[0] = heap[heap.Count - 1];
            heap.RemoveAt(heap.Count - 1);

            if (heap.Count > 0)
                HeapifyDown(0);

            return result;
        }

        private void HeapifyUp(int index)
        {
            while (index > 0)
            {
                int parent = (index - 1) / 2;
                if (heap[parent].cost <= heap[index].cost)
                    break;

                Swap(index, parent);
                index = parent;
            }
        }

        private void HeapifyDown(int index)
        {
            int last = heap.Count - 1;
            while (true)
            {
                int left = 2 * index + 1;
                int right = 2 * index + 2;
                int smallest = index;

                if (left <= last && heap[left].cost < heap[smallest].cost)
                    smallest = left;
                if (right <= last && heap[right].cost < heap[smallest].cost)
                    smallest = right;

                if (smallest == index) break;

                Swap(index, smallest);
                index = smallest;
            }
        }

        private void Swap(int i, int j)
        {
            var temp = heap[i];
            heap[i] = heap[j];
            heap[j] = temp;
        }
    }

    /// <summary>
    /// Finds the shortest path from <paramref name="start"/> to <paramref name="goal"/> using Dijkstra's algorithm.
    /// Handles walls with varying costs and vents (teleportation points).
    /// </summary>
    /// <param name="start">The starting grid position.</param>
    /// <param name="goal">The target grid position.</param>
    /// <param name="mapData">Interface providing map information, wall costs, and vent data.</param>
    /// <returns>A list of grid positions representing the shortest path, or null if no path exists.</returns>
    public static List<Vector2Int> FindShortestPath(Vector2Int start, Vector2Int goal, IMapData mapData)
    {
        if (start == goal)
            return new List<Vector2Int> { start };

        // Use min-heap for efficient priority queue operations
        var priorityQueue = new MinHeap();
        priorityQueue.Enqueue(start, 0f);

        var costSoFar = new Dictionary<Vector2Int, float> { [start] = 0f };
        var previous = new Dictionary<Vector2Int, Vector2Int>();
        var visited = new HashSet<Vector2Int>(); // For lazy deletion

        Vector2Int[] directions = new Vector2Int[] { Vector2Int.up, Vector2Int.down, Vector2Int.left, Vector2Int.right };

        while (priorityQueue.Count > 0)
        {
            var (current, currentCost) = priorityQueue.Dequeue();

            // Lazy deletion: skip if already processed with better cost
            if (visited.Contains(current))
                continue;

            visited.Add(current);

            if (current == goal)
                return ReconstructPath(previous, start, goal);

            foreach (var direction in directions)
            {
                Vector2Int neighbor = current + direction;

                if (neighbor.x < 0 || neighbor.x >= mapData.Width || neighbor.y < 0 || neighbor.y >= mapData.Height)
                    continue;

                float moveCost = GetMovementCost(current, neighbor, mapData);
                if (float.IsInfinity(moveCost))
                    continue;

                float newCost = costSoFar[current] + moveCost;
                if (!costSoFar.ContainsKey(neighbor) || newCost < costSoFar[neighbor])
                {
                    costSoFar[neighbor] = newCost;
                    previous[neighbor] = current;
                    priorityQueue.Enqueue(neighbor, newCost);
                }
            }

            // Handle teleportation via vents
            if (mapData.HasVent(current.x, current.y))
            {
                float ventCost = mapData.GetVentCost(current.x, current.y);
                foreach (var otherVent in mapData.GetOtherVentPositions(current))
                {
                    float newCost = costSoFar[current] + ventCost;
                    if (!costSoFar.ContainsKey(otherVent) || newCost < costSoFar[otherVent])
                    {
                        costSoFar[otherVent] = newCost;
                        previous[otherVent] = current;
                        priorityQueue.Enqueue(otherVent, newCost);
                    }
                }
            }
        }

        Debug.LogWarning("Path not found!");
        return null;
    }

    /// <summary>
    /// Reconstructs the path from start to goal using the dictionary of previous nodes.
    /// </summary>
    /// <param name="previous">A mapping of each node to its predecessor on the shortest path.</param>
    /// <param name="start">The starting node.</param>
    /// <param name="goal">The goal node.</param>
    /// <returns>A list of nodes representing the reconstructed path.</returns>
    private static List<Vector2Int> ReconstructPath(Dictionary<Vector2Int, Vector2Int> previous, Vector2Int start, Vector2Int goal)
    {
        List<Vector2Int> path = new List<Vector2Int>();
        Vector2Int current = goal;

        while (current != start)
        {
            path.Add(current);
            current = previous[current];
        }

        path.Add(start);
        path.Reverse();
        return path;
    }

    /// <summary>
    /// Calculates the movement cost from one cell to an adjacent cell, accounting for walls and climbing costs.
    /// </summary>
    /// <param name="from">The origin cell.</param>
    /// <param name="to">The destination cell.</param>
    /// <param name="mapData">Interface providing wall cost information.</param>
    /// <returns>The cost to move from <paramref name="from"/> to <paramref name="to"/>, or PositiveInfinity if blocked.</returns>
    public static float GetMovementCost(Vector2Int from, Vector2Int to, IMapData mapData)
    {
        float baseCost = 1f;

        if (from.x == to.x)
        {
            int checkY = Mathf.Max(from.y, to.y);
            float wallCost = mapData.GetHorizontalWallCost(from.x, checkY);
            if (float.IsInfinity(wallCost)) return float.PositiveInfinity;
            baseCost += wallCost - 1f;
        }
        else
        {
            int checkX = Mathf.Max(from.x, to.x);
            float wallCost = mapData.GetVerticalWallCost(checkX, from.y);
            if (float.IsInfinity(wallCost)) return float.PositiveInfinity;
            baseCost += wallCost - 1f;
        }

        return baseCost;
    }

    /// <summary>
    /// Determines if movement from one cell to another is blocked.
    /// </summary>
    /// <param name="from">The starting cell.</param>
    /// <param name="to">The destination cell.</param>
    /// <param name="mapData">Map interface to check walls.</param>
    /// <returns>True if movement is blocked; otherwise false.</returns>
    public static bool IsMovementBlocked(Vector2Int from, Vector2Int to, IMapData mapData)
    {
        return float.IsInfinity(GetMovementCost(from, to, mapData));
    }

}