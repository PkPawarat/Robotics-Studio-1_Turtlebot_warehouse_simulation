
#include <iostream>
#include <vector>
#include "ControllerInterface/library/controller.h"
#include "ControllerInterface/library/controllerinterface.h"
#include "ControllerInterface/library/pathfinding.h"
#include "ControllerInterface/library/ROSNode.h"
#include "ControllerInterface/library/sensor.h"
#include "MissionInterface/library/missionInterface.h"
#include "MissionInterface/library/mission.h"



void printGrid(const std::vector<std::vector<int>>& grid, const std::vector<Cell>& path = std::vector<Cell>()) {
    // Create a copy of the grid to mark the path
    std::vector<std::vector<std::string>> pathGrid(grid.size(), std::vector<std::string>(grid[0].size()));

    for (size_t i = 0; i < pathGrid.size(); i++) {
        for (size_t j = 0; j < pathGrid[0].size(); j++) {
            pathGrid[i][j] = std::to_string(grid[i][j]); // Initialize with obstacles or empty spaces
        }
    }

    for (const Cell& cell : path) {
        pathGrid[cell.row][cell.col] = "*"; // Mark path cells with asterisk
    }

    // Print the path-marked grid
    for (size_t i = 0; i < pathGrid.size(); i++) {
        for (size_t j = 0; j < pathGrid[0].size(); j++) {
            std::cout << pathGrid[i][j] << " ";
        }
        std::cout << std::endl;
    }
}

// Function to trim leading and trailing spaces from a string
std::string trim(const std::string& s) {
    size_t start = s.find_first_not_of(" \t\r\n");
    size_t end = s.find_last_not_of(" \t\r\n");
    if (start == std::string::npos || end == std::string::npos) {
        // String is empty or contains only spaces
        return "";
    }
    return s.substr(start, end - start + 1);
}


int main() {
    // Replace "your_excel_file.xlsx" with the path to your Excel file
    std::string excelFilePath = "/home/pk/git/Robotics-Studio-1/ExcelFile/PlatLocation1.csv";
    // /home/pk/git/Robotics-Studio-1/ExcelFile
    // /home/pawarat/git/Robotics-Studio-1/ExcelFile

    // Create an instance of the platFinding class
    platFinding pathFinder(excelFilePath);

    // Define the start and goal cells
    Cell start = {0, 0};
    Cell goal = {5, 5};

    // Find the path from start to goal
    std::vector<Cell> path = pathFinder.FindPath(start, goal);

    // Print the path
    std::cout << "Path from (" << start.row << "," << start.col << ") to (" << goal.row << "," << goal.col << "):" << std::endl;
    for (const Cell& cell : path) {
        std::cout << "(" << cell.row << "," << cell.col << ") ";
    }
    std::cout << std::endl;

    printGrid(pathFinder.grid);
    return 0;
}

