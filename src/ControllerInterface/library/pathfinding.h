#ifndef PATHFINDING
#define PATHFINDING
#include <vector>
#include <unordered_map>
#include <unordered_set>

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

struct Cell {
    int Row;
    int Col;
};

struct CellHash {
    std::size_t operator()(const Cell& cell) const {
        // Combine the hash codes of the Row and Col members
        return std::hash<int>()(cell.Row) ^ std::hash<int>()(cell.Col);
    }
};

struct CellEqual {
    bool operator()(const Cell& lhs, const Cell& rhs) const {
        return lhs.Row == rhs.Row && lhs.Col == rhs.Col;
    }
};

class pathfinding {
    private:
        int** grid;
        int rows;
        int columns;

    public:
        pathfinding(int** grid, int rows, int columns);
        ~pathfinding();
        
        std::vector<Cell> FindPath(const Cell& start, const Cell& goal);

    private:
        std::vector<Cell> GetNeighbors(const Cell& cell);
        double CalculateHeuristic(const Cell& cell, const Cell& goal);
        std::vector<Cell> ReconstructPath(std::unordered_map<Cell, Cell, CellHash, CellEqual>& cameFrom, Cell current);
};
#endif // PATHFINDING
