
#include <iostream>
#include <vector>
#include "ControllerInterface/library/controller.h"
#include "ControllerInterface/library/controllerinterface.h"
#include "ControllerInterface/library/pathfinding.h"
#include "ControllerInterface/library/ROSEnvironment.h"
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

// // Function to convert cell values to symbols
// std::string convertCellValue(const std::string& cellValue) {
//     // Split the cellValue into prefix and value
//     std::istringstream iss(cellValue);
//     std::string prefix, value;
    
//     if (std::getline(iss, prefix, ' ')) {
//         std::getline(iss, value);
//         if (value.empty()) {
//             value = cellValue;
//         }
//     } else {
//         // No comma-separated prefix found, use the entire cellValue as value
//         value = cellValue;
//     }

//     // Remove spaces from the value
//     value.erase(std::remove_if(value.begin(), value.end(), ::isspace), value.end());

//     static std::unordered_map<std::string, std::string> cellMappings = {
//         {"L", "←"},
//         {"R", "→"},
//         {"U", "↑"},
//         {"D", "↓"},
//         {"U&D", "↕"},
//         {"L&R", "↔"},
//         {"U&L", "←↑"},
//         {"U&R", "↑→"},
//         {"D&L", "←↓"},
//         {"D&R", "↓→"},
//         {"U&D&L", "←↕"},
//         {"U&D&R", "↕→"},
//         {"U&L&R", "↑↔"},
//         {"D&L&R", "↓↔"},
//         {"U&D&L&R", "+"}
//     };

//     // Check if the prefix is one of P, W, or C
//     if (prefix == "P" || prefix == "W" || prefix == "C") {
//         auto it = cellMappings.find(value);
//         if (it != cellMappings.end()) {
//             return prefix + it->second;
//         } else {
//             return cellValue; // Return the original value if not found in mappings
//         }
//     } else {
//         auto it = cellMappings.find(value);
//         if (it != cellMappings.end()) {
//             return it->second;
//         } else {
//             return value; // Return the original value if not found in mappings
//         }
//     }
// }


// int main() {
//     // Open the CSV file
//     std::ifstream wb("/home/pawarat/Robotics-Studio-1/PlatLocation1.csv");

//     // Check if the file opened successfully
//     if (!wb.is_open()) {
//         std::cerr << "Error: Could not open the CSV file." << std::endl;
//         return 1;
//     }

//     // Define variables to store the CSV data
//     std::vector<std::vector<std::string>> grid;
//     std::string line;

//     // Read and process each line of the CSV file
//     while (std::getline(wb, line)) {
//         std::vector<std::string> rowData;
//         std::istringstream ss(line);
//         std::string cell;

//         // Split each line into comma-separated values
//         while (std::getline(ss, cell, ',')) {
//             // Trim leading and trailing spaces from the cell value
//             cell = trim(cell);
//             // Convert the cell value to the corresponding symbol
//             cell = convertCellValue(cell);
//             rowData.push_back(cell);
//         }

//         // Add the row data to the grid
//         grid.push_back(rowData);
//     }

//     // Close the file
//     wb.close();

//     // Print the grid
//     std::cout << "Grid read from CSV file:" << std::endl;
//     for (const std::vector<std::string>& row : grid) {
//         for (const std::string& cellValue : row) {
//             std::cout << cellValue << " ";
//         }
//         std::cout << std::endl;
//     }

//     return 0;
// }

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


// int main() {
//     // std::ifstream wb("/home/pawarat/Robotics-Studio-1/PlatLocation1.csv");
    
    
    
    
    
    
//     // Create an instance of platFinding with the path to your Excel file
//     platFinding finder("/home/pawarat/Robotics-Studio-1/PlatLocation1.csv");

//     // Get the grid from platFinding
//     std::vector<std::vector<int>> grid = finder.grid;

//     // Print the grid
//     std::cout << "Grid read from Excel file:" << std::endl;
//     for (const std::vector<int>& row : grid) {
//         for (int cellValue : row) {
//             std::cout << cellValue << " ";
//         }
//         std::cout << std::endl;
//     }

//     // Define the start and goal cells
//     Cell start = {0, 0};  // Replace with your actual start cell
//     Cell goal = {4, 4};   // Replace with your actual goal cell

//     // Find the path
//     std::vector<Cell> path = finder.FindPath(start, goal);

//     // Print the path
//     if (!path.empty()) {
//         std::cout << "Path found:" << std::endl;
//         for (const Cell& cell : path) {
//             std::cout << "(" << cell.row << ", " << cell.col << ") -> ";
//         }
//         std::cout << "Goal" << std::endl;
//     } else {
//         std::cout << "No path found." << std::endl;
//     }

//     return 0;
// }
