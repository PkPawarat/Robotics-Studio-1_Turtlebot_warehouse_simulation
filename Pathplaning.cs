using System;
using System.Collections.Generic;

public class Cell
{
    public int Row { get; set; }
    public int Col { get; set; }
}

public class AStar
{
    private int[,] grid;
    private int rows;
    private int columns;

    public AStar(int[,] grid)
    {
        this.grid = grid;
        rows = grid.GetLength(0);
        columns = grid.GetLength(1);
    }

    private List<Cell> GetNeighbors(Cell cell)
    {
        List<Cell> neighbors = new List<Cell>
        {
            new Cell { Row = cell.Row + 1, Col = cell.Col },
            new Cell { Row = cell.Row - 1, Col = cell.Col },
            new Cell { Row = cell.Row, Col = cell.Col + 1 },
            new Cell { Row = cell.Row, Col = cell.Col - 1 }
        };

        // Filter out cells that are out of bounds or obstacles
        neighbors.RemoveAll(n => n.Row < 0 || n.Row >= rows || n.Col < 0 || n.Col >= columns || grid[n.Row, n.Col] == 1);

        return neighbors;
    }

    private double CalculateHeuristic(Cell cell, Cell goal)
    {
        return Math.Sqrt(Math.Pow(cell.Row - goal.Row, 2) + Math.Pow(cell.Col - goal.Col, 2));
    }

    public List<Cell> FindPath(Cell start, Cell goal)
    {
        var openSet = new PriorityQueue<Cell>();
        var cameFrom = new Dictionary<Cell, Cell>();
        var gScore = new Dictionary<Cell, double>();
        var fScore = new Dictionary<Cell, double>();
        var closedSet = new HashSet<Cell>();

        openSet.Enqueue(start, 0);
        gScore[start] = 0;
        fScore[start] = CalculateHeuristic(start, goal);

        while (openSet.Count > 0)
        {
            Cell current = openSet.Dequeue();

            if (current.Row == goal.Row && current.Col == goal.Col)
            {
                return ReconstructPath(cameFrom, current);
            }

            closedSet.Add(current);

            foreach (Cell neighbor in GetNeighbors(current))
            {
                if (closedSet.Contains(neighbor))
                    continue;

                double tentativeGScore = gScore[current] + CalculateHeuristic(current, neighbor);

                if (!gScore.ContainsKey(neighbor) || tentativeGScore < gScore[neighbor])
                {
                    cameFrom[neighbor] = current;
                    gScore[neighbor] = tentativeGScore;
                    fScore[neighbor] = gScore[neighbor] + CalculateHeuristic(neighbor, goal);

                    if (!openSet.Contains(neighbor))
                        openSet.Enqueue(neighbor, fScore[neighbor]);
                }
            }
        }

        return null;
    }

    private List<Cell> ReconstructPath(Dictionary<Cell, Cell> cameFrom, Cell current)
    {
        List<Cell> path = new List<Cell> { current };

        while (cameFrom.ContainsKey(current))
        {
            current = cameFrom[current];
            path.Add(current);
        }

        path.Reverse();
        return path;
    }
}

public class PriorityQueue<T>
{
    private List<(T item, double priority)> elements = new List<(T item, double priority)>();

    public int Count => elements.Count;

    public void Enqueue(T item, double priority)
    {
        elements.Add((item, priority));
        int index = elements.Count - 1;

        while (index > 0)
        {
            int parentIndex = (index - 1) / 2;

            if (elements[parentIndex].priority <= elements[index].priority)
                break;

            (T tempItem, double tempPriority) = elements[index];
            elements[index] = elements[parentIndex];
            elements[parentIndex] = (tempItem, tempPriority);

            index = parentIndex;
        }
    }

    public T Dequeue()
    {
        T topItem = elements[0].item;
        elements[0] = elements[elements.Count - 1];
        elements.RemoveAt(elements.Count - 1);

        int index = 0;

        while (true)
        {
            int child1Index = 2 * index + 1;
            int child2Index = 2 * index + 2;

            if (child1Index >= elements.Count)
                break;

            int minChildIndex = child2Index < elements.Count && elements[child2Index].priority < elements[child1Index].priority ? child2Index : child1Index;

            if (elements[index].priority <= elements[minChildIndex].priority)
                break;

            (T tempItem, double tempPriority) = elements[index];
            elements[index] = elements[minChildIndex];
            elements[minChildIndex] = (tempItem, tempPriority);

            index = minChildIndex;
        }

        return topItem;
    }

    public bool Contains(T item)
    {
        foreach (var (existingItem, _) in elements)
        {
            if (EqualityComparer<T>.Default.Equals(existingItem, item))
                return true;
        }

        return false;
    }
}

public class Program
{
    public static void Main()
    {
        int[,] grid = new int[,]
        {
            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
            { 0, 1, 0, 1, 1, 1, 0, 1, 1, 0 },
            { 0, 1, 0, 1, 1, 1, 0, 1, 1, 0 },
            { 0, 0, 0, 1, 1, 1, 0, 1, 1, 0 },
            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
            { 0, 1, 0, 1, 1, 1, 0, 1, 1, 0 },
            { 0, 1, 0, 1, 1, 1, 0, 1, 1, 0 },
            { 0, 1, 0, 1, 1, 1, 0, 1, 1, 0 },
            { 0, 0, 0, 1, 1, 1, 0, 1, 1, 0 },
            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
        };

        AStar aStar = new AStar(grid);
        List<Cell> path = aStar.FindPath(new Cell { Row = 0, Col = 0 }, new Cell { Row = 9, Col = 9 });

        if (path != null)
        {
            Console.WriteLine("Path found.");

            // Create a copy of the grid to mark the path
            string[,] pathGrid = new string[grid.GetLength(0), grid.GetLength(1)];

            for (int i = 0; i < pathGrid.GetLength(0); i++)
            {
                for (int j = 0; j < pathGrid.GetLength(1); j++)
                {
                    pathGrid[i, j] = grid[i, j] == 0 ? "0" : "1"; // Initialize with obstacles or empty spaces
                }
            }

            foreach (Cell cell in path)
            {
                pathGrid[cell.Row, cell.Col] = "*"; // Mark path cells with asterisk
            }

            // Print the path-marked grid
            for (int i = 0; i < pathGrid.GetLength(0); i++)
            {
                for (int j = 0; j < pathGrid.GetLength(1); j++)
                {
                    Console.Write(pathGrid[i, j] + " ");
                }
                Console.WriteLine();
            }
        }
        else
        {
            Console.WriteLine("No path found.");
        }
    }
}
