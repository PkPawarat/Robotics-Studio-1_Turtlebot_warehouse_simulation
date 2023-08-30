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

class AStar {
private:
    int** grid;
    int rows;
    int columns;

public:
    AStar(int** grid, int rows, int columns);
    ~AStar();
    
    std::vector<Cell> FindPath(const Cell& start, const Cell& goal);

private:
    std::vector<Cell> GetNeighbors(const Cell& cell);
    double CalculateHeuristic(const Cell& cell, const Cell& goal);
    std::vector<Cell> ReconstructPath(std::unordered_map<Cell, Cell>& cameFrom, Cell current);
};
#endif // PATHFINDING
