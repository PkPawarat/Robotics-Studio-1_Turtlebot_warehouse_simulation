#include "library/pathfinding.h"
#include <cmath>
#include <queue>
#include <functional>
#include <algorithm>
#include <utility>

// Keep only the headers needed
#include <vector>
#include "pfms_types.h"
#include "ros/ros.h"
#include <atomic>
#include <mutex>

#include "visualization_msgs/MarkerArray.h"
#include "std_srvs/SetBool.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseArray.h"

class PriorityQueue {
private:
    std::priority_queue<std::pair<double, Cell>, std::vector<std::pair<double, Cell>>, std::greater<>> elements;

public:
    int Count() const {
        return elements.size();
    }

    void Enqueue(const Cell& item, double priority) {
        elements.push(std::make_pair(priority, item));
    }

    Cell Dequeue() {
        Cell topItem = elements.top().second;
        elements.pop();
        return topItem;
    }

    bool Contains(const Cell& item) {
        return std::any_of(elements.c.begin(), elements.c.end(),
            [&item](const std::pair<double, Cell>& existingItem) {
                return existingItem.second.Row == item.Row && existingItem.second.Col == item.Col;
            });
    }
};

AStar::AStar(int** grid, int rows, int columns)
    : grid(grid), rows(rows), columns(columns) {}

AStar::~AStar() {
    // Deallocate grid memory
    for (int i = 0; i < rows; ++i) {
        delete[] grid[i];
    }
    delete[] grid;
}

std::vector<Cell> AStar::GetNeighbors(const Cell& cell) {
    std::vector<Cell> neighbors = {
        {cell.Row + 1, cell.Col},
        {cell.Row - 1, cell.Col},
        {cell.Row, cell.Col + 1},
        {cell.Row, cell.Col - 1}
    };

    neighbors.erase(std::remove_if(neighbors.begin(), neighbors.end(),
        [this](const Cell& n) {
            return n.Row < 0 || n.Row >= rows || n.Col < 0 || n.Col >= columns || grid[n.Row][n.Col] == 1;
        }),
        neighbors.end());

    return neighbors;
}

double AStar::CalculateHeuristic(const Cell& cell, const Cell& goal) {
    return std::sqrt(std::pow(cell.Row - goal.Row, 2) + std::pow(cell.Col - goal.Col, 2));
}

std::vector<Cell> AStar::FindPath(const Cell& start, const Cell& goal) {
    PriorityQueue openSet;
    std::unordered_map<Cell, Cell> cameFrom;
    std::unordered_map<Cell, double> gScore;
    std::unordered_map<Cell, double> fScore;
    std::unordered_set<Cell> closedSet;

    openSet.Enqueue(start, 0);
    gScore[start] = 0;
    fScore[start] = CalculateHeuristic(start, goal);

    while (openSet.Count() > 0) {
        Cell current = openSet.Dequeue();

        if (current.Row == goal.Row && current.Col == goal.Col) {
            return ReconstructPath(cameFrom, current);
        }

        closedSet.insert(current);

        for (const Cell& neighbor : GetNeighbors(current)) {
            if (closedSet.find(neighbor) != closedSet.end())
                continue;

            double tentativeGScore = gScore[current] + CalculateHeuristic(current, neighbor);

            if (gScore.find(neighbor) == gScore.end() || tentativeGScore < gScore[neighbor]) {
                cameFrom[neighbor] = current;
                gScore[neighbor] = tentativeGScore;
                fScore[neighbor] = gScore[neighbor] + CalculateHeuristic(neighbor, goal);

                if (!openSet.Contains(neighbor))
                    openSet.Enqueue(neighbor, fScore[neighbor]);
            }
        }
    }

    return std::vector<Cell>();
}

std::vector<Cell> AStar::ReconstructPath(std::unordered_map<Cell, Cell>& cameFrom, Cell current) {
    std::vector<Cell> path = { current };

    while (cameFrom.find(current) != cameFrom.end()) {
        current = cameFrom[current];
        path.push_back(current);
    }

    std::reverse(path.begin(), path.end());
    return path;
}
