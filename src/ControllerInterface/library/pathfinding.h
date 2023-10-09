#ifndef PATHFINDING_H
#define PATHFINDING_H

#include <vector>
#include <unordered_map>
#include <iostream>
#include <vector>
#include <queue>
#include <map>  // Include the <map> header for std::map
#include <set>  // Include the <set> header for std::set
#include <cmath>
#include <algorithm>

#include <iostream>
#include <fstream>
#include <sstream> // Include this for std::stringstream
#include <string>
#include <vector>


struct Cell {
    int row;
    int col;
};

struct CellCompare {
    bool operator()(const Cell& lhs, const Cell& rhs) const {
        return std::tie(lhs.row, lhs.col) < std::tie(rhs.row, rhs.col);
    }
};

struct CostCellPair {
    double cost;
    Cell cell;
    CostCellPair(double _cost, const Cell& _cell) : cost(_cost), cell(_cell) {}

    bool operator>(const CostCellPair& other) const {
        return cost > other.cost;
    }
    bool operator<(const CostCellPair& other) const {
        return cost < other.cost;
    }
};

class platFinding {
private:
    int rows;
    int columns;
public:
    platFinding(const std::string& excelFilePath);
    std::vector<Cell> FindPath(Cell start, Cell goal);
    std::vector<Cell> GetNeighbors(Cell cell);
    double CalculateHeuristic(Cell cell, Cell goal);
    std::vector<Cell> ReconstructPath(std::map<Cell, Cell, CellCompare> cameFrom, Cell current) ;
    std::vector<std::vector<int>> grid;
    std::string convertCellValue(const std::string& cellValue); // Add the function here
    int symbolToInt(const std::string& symbol);
    void printGrid(const std::vector<std::vector<int>>& grid, const std::vector<Cell>& path = std::vector<Cell>());
    std::string trim(const std::string& s);
};


#endif // PATHFINDING_H
