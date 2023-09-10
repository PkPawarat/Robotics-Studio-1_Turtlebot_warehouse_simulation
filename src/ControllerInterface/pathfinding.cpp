
// #include "library/pathfinding.h"
// #include <iostream>
// #include <vector>
// #include <queue>
// #include <map>  // Include the <map> header for std::map
// #include <set>  // Include the <set> header for std::set
// #include <cmath>
// #include <algorithm>

// #include <iostream>
// #include <fstream>
// #include <sstream> // Include this for std::stringstream
// #include <string>
// #include <vector>
// // #include <libxl.h>

// // platFinding::platFinding(const std::string& excelFilePath) {
// //     std::ifstream wb(excelFilePath);  // Use ifstream to read the Excel file
// //     std::string line;
// //     std::stringstream ss;

// //     while (std::getline(wb, line)) {
// //         ss.clear();    // Clear any error flags
// //         ss.str(line);  // Set the content of ss to the current line

// //         std::vector<int> rowData;
// //         std::string cell;

// //         while (std::getline(ss, cell, ',')) {
// //             try {
// //                 int cellValue = std::stoi(cell);
// //                 rowData.push_back(cellValue);
// //             } catch (const std::exception& e) {
// //                 std::cerr << "Error parsing cell value: " << e.what() << std::endl;
// //                 rowData.push_back(0); // Handle the error case
// //             }
// //         }

// //         if (columns == 0) {
// //             columns = static_cast<int>(rowData.size());
// //         }

// //         if (rowData.size() != static_cast<size_t>(columns)) {
// //             std::cerr << "Inconsistent number of columns in the CSV file." << std::endl;
// //             break;
// //         }

// //         grid.push_back(rowData);
// //         ++rows;
// //     }

// //     wb.close();
// // }
// platFinding::platFinding(const std::string& excelFilePath) {
//     std::ifstream wb(excelFilePath);  // Use ifstream to read the Excel file
//     std::string line;
//     std::stringstream ss;

//     while (std::getline(wb, line)) {
//         ss.clear();    // Clear any error flags
//         ss.str(line);  // Set the content of ss to the current line

//         std::vector<int> rowData;
//         std::string cell;

//         while (std::getline(ss, cell, ',')) {
//             try {
//                 // Use the convertCellValue function to convert the cell value
//                 std::string check = convertCellValue(cell);
//                 // symbolToInt(check);
//                 int cellValue = symbolToInt(check);
//                 // std::cerr << cellValue << std::endl;
//                 rowData.push_back(cellValue);
//             } catch (const std::exception& e) {
//                 std::cerr << "Error parsing cell value: " << e.what() << std::endl;
//                 rowData.push_back(0); // Handle the error case
//             }
//         }

//         if (columns == 0) {
//             columns = static_cast<int>(rowData.size());
//         }

//         if (rowData.size() != static_cast<size_t>(columns)) {
//             std::cerr << "Inconsistent number of columns in the CSV file." << std::endl;
//             break;
//         }

//         grid.push_back(rowData);
//         ++rows;
//     }

//     wb.close();
// }

// std::string platFinding::convertCellValue(const std::string& cellValue) {
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
//             // std::cerr << "" << prefix + it->second << std::endl;

//             return prefix + it->second;
//         } else {
//             // std::cerr << "" << cellValue << std::endl;
//             return cellValue; // Return the original value if not found in mappings
//         }
//     } else {
//         auto it = cellMappings.find(value);
//         if (it != cellMappings.end()) {
//             // std::cerr << "" << it->second << std::endl;

//             return it->second;
//         } else {
//             // std::cerr << "" << value << std::endl;

//             return value; // Return the original value if not found in mappings
//         }
//     }
// }

// // Function to convert a special symbol to an integer
// int platFinding::symbolToInt(const std::string& symbol) {
//     static std::unordered_map<std::string, int> symbolMappings = {
//         {"←", 1},
//         {"→", 2},
//         {"↑", 3},
//         {"↓", 4},
//         {"↕", 5},
//         {"↔", 6},
//         {"←↑", 7},
//         {"↑→", 8},
//         {"←↓", 9},
//         {"↓→", 10},
//         {"←↕", 11},
//         {"↕→", 12},
//         {"↑↔", 13},
//         {"↓↔", 14},
//         {"+", 15}
//     };
//     char firstChar = symbol[0];
        
//         if (firstChar == 'P' || firstChar == 'W' || firstChar == 'C') {
//             std::string symbolWithoutPrefix = symbol.substr(1); // Remove the prefix
//             auto it = symbolMappings.find(symbolWithoutPrefix);
            
//             if (it != symbolMappings.end()) {
//                 int adjustedValue = it->second;
                
//                 // Adjust the value based on the prefix
//                 if (firstChar == 'P') {
//                     adjustedValue += 100;
//                 } else if (firstChar == 'W') {
//                     adjustedValue += 200;
//                 } else if (firstChar == 'C') {
//                     adjustedValue += 300;
//                 }
                
//                 return adjustedValue;
//             }
//         } else {
//             auto it = symbolMappings.find(symbol);
//             if (it != symbolMappings.end()) {
//                 return it->second;
//             }
//         }

//         // Return 0 for symbols that are not found
//         return 0;
    
// }

// std::vector<Cell> platFinding::GetNeighbors(Cell cell) {
//     std::vector<Cell> neighbors;

//     // Check if the cell is in the cellMappings and determine which directions are allowed
//     std::string cellValue = std::to_string(grid[cell.row][cell.col]);
//     std::istringstream iss(cellValue);
//     std::string prefix, directions;

//     if (std::getline(iss, prefix, ',')) {
//         std::getline(iss, directions);
//     } else {
//         directions = cellValue;
//     }

//     // Remove spaces from directions
//     directions.erase(std::remove_if(directions.begin(), directions.end(), ::isspace), directions.end());

//     // Check each direction and add neighbors accordingly
//     for (char direction : directions) {
//         switch (direction) {
//             case 'L':
//                 neighbors.push_back({cell.row, cell.col - 1});
//                 break;
//             case 'R':
//                 neighbors.push_back({cell.row, cell.col + 1});
//                 break;
//             case 'U':
//                 neighbors.push_back({cell.row - 1, cell.col});
//                 break;
//             case 'D':
//                 neighbors.push_back({cell.row + 1, cell.col});
//                 break;
//         }
//     }

//     // Use std::remove_if from the <algorithm> header to filter out invalid neighbors
//     neighbors.erase(std::remove_if(neighbors.begin(), neighbors.end(),
//         [&](const Cell& n) {
//             return (n.row < 0 || n.row >= rows || n.col < 0 || n.col >= columns);
//         }), neighbors.end());

//     return neighbors;
// }



// double platFinding::CalculateHeuristic(Cell cell, Cell goal) {
//     return std::sqrt(std::pow(cell.row - goal.row, 2) + std::pow(cell.col - goal.col, 2));
// }


// std::vector<Cell> platFinding::FindPath(Cell start, Cell goal) {
//     std::priority_queue<CostCellPair> openSet; // Use the custom struct

//     std::map<Cell, Cell, CellCompare> cameFrom;
//     std::map<Cell, double, CellCompare> gScore;
//     std::map<Cell, double, CellCompare> fScore;
//     std::set<Cell, CellCompare> closedSet;

//     openSet.push(CostCellPair(0, start));
//     gScore[start] = 0;
//     fScore[start] = CalculateHeuristic(start, goal);

//     while (!openSet.empty()) {
//         CostCellPair currentPair = openSet.top();
//         openSet.pop();
//         Cell current = currentPair.cell;

//         if (current.row == goal.row && current.col == goal.col) {
//             return ReconstructPath(cameFrom, current);
//         }

//         closedSet.insert(current);

//         for (const Cell& neighbor : GetNeighbors(current)) {
//             if (closedSet.count(neighbor) > 0)
//                 continue;

//             double tentativeGScore = gScore[current] + CalculateHeuristic(current, neighbor);

//             if (gScore.find(neighbor) == gScore.end() || tentativeGScore < gScore[neighbor]) {
//                 cameFrom[neighbor] = current;
//                 gScore[neighbor] = tentativeGScore;
//                 fScore[neighbor] = gScore[neighbor] + CalculateHeuristic(neighbor, goal);

//                 openSet.push(CostCellPair(fScore[neighbor], neighbor));
//             }
//         }
//     }

//     return std::vector<Cell>(); // No path found
// }




// std::vector<Cell> platFinding::ReconstructPath(std::map<Cell, Cell, CellCompare> cameFrom, Cell current) {
//     std::vector<Cell> path;
//     path.push_back(current);

//     while (cameFrom.find(current) != cameFrom.end()) {
//         current = cameFrom[current];
//         path.push_back(current);
//     }

//     std::reverse(path.begin(), path.end());
//     return path;
// }


#include "library/pathfinding.h"
#include <iostream>
#include <vector>
#include <queue>
#include <map>
#include <set>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

platFinding::platFinding(const std::string& excelFilePath) {
    std::ifstream wb(excelFilePath);
    std::string line;
    std::stringstream ss;

    while (std::getline(wb, line)) {
        ss.clear();
        ss.str(line);

        std::vector<int> rowData;
        std::string cell;

        while (std::getline(ss, cell, ',')) {
            try {
                std::string check = convertCellValue(cell);
                int cellValue = symbolToInt(check);
                rowData.push_back(cellValue);
            } catch (const std::exception& e) {
                std::cerr << "Error parsing cell value: " << e.what() << std::endl;
                rowData.push_back(0);
            }
        }

        if (columns == 0) {
            columns = static_cast<int>(rowData.size());
        }

        if (rowData.size() != static_cast<size_t>(columns)) {
            std::cerr << "Inconsistent number of columns in the CSV file." << std::endl;
            break;
        }

        grid.push_back(rowData);
        ++rows;
    }

    wb.close();
}

std::string platFinding::convertCellValue(const std::string& cellValue) {
    std::istringstream iss(cellValue);
    std::string prefix, value;

    if (std::getline(iss, prefix, ' ')) {
        std::getline(iss, value);
        if (value.empty()) {
            value = cellValue;
        }
    } else {
        value = cellValue;
    }

    value.erase(std::remove_if(value.begin(), value.end(), ::isspace), value.end());

    static std::unordered_map<std::string, std::string> cellMappings = {
        {"L", "←"},
        {"R", "→"},
        {"U", "↑"},
        {"D", "↓"},
        {"U&D", "↕"},
        {"L&R", "↔"},
        {"U&L", "←↑"},
        {"U&R", "↑→"},
        {"D&L", "←↓"},
        {"D&R", "↓→"},
        {"U&D&L", "←↕"},
        {"U&D&R", "↕→"},
        {"U&L&R", "↑↔"},
        {"D&L&R", "↓↔"},
        {"U&D&L&R", "+"}
    };

    if (prefix == "P" || prefix == "W" || prefix == "C") {
        auto it = cellMappings.find(value);
        if (it != cellMappings.end()) {
            return prefix + it->second;
        } else {
            return cellValue;
        }
    } else {
        auto it = cellMappings.find(value);
        if (it != cellMappings.end()) {
            return it->second;
        } else {
            return value;
        }
    }
}

int platFinding::symbolToInt(const std::string& symbol) {
    static std::unordered_map<std::string, int> symbolMappings = {
        {"←", 1},
        {"→", 2},
        {"↑", 3},
        {"↓", 4},
        {"↕", 5},
        {"↔", 6},
        {"←↑", 7},
        {"↑→", 8},
        {"←↓", 9},
        {"↓→", 10},
        {"←↕", 11},
        {"↕→", 12},
        {"↑↔", 13},
        {"↓↔", 14},
        {"+", 15}
    };

    char firstChar = symbol[0];

    if (firstChar == 'P' || firstChar == 'W' || firstChar == 'C') {
        std::string symbolWithoutPrefix = symbol.substr(1);
        auto it = symbolMappings.find(symbolWithoutPrefix);

        if (it != symbolMappings.end()) {
            int adjustedValue = it->second;

            if (firstChar == 'P') {
                adjustedValue += 100;
            } else if (firstChar == 'W') {
                adjustedValue += 200;
            } else if (firstChar == 'C') {
                adjustedValue += 300;
            }

            return adjustedValue;
        }
    } else {
        auto it = symbolMappings.find(symbol);
        if (it != symbolMappings.end()) {
            return it->second;
        }
    }

    return 0;
}


//TODO check the correct direction 
std::vector<Cell> platFinding::GetNeighbors(Cell cell) {
    std::vector<Cell> neighbors;
    std::string cellValue = std::to_string(grid[cell.row][cell.col]);
    std::istringstream iss(cellValue);
    std::string prefix, directions;

    if (std::getline(iss, prefix, ',')) {
        std::getline(iss, directions);
    } else {
        directions = cellValue;
    }

    directions.erase(std::remove_if(directions.begin(), directions.end(), ::isspace), directions.end());

    for (char direction : directions) {
        switch (direction) {
            case 'L':
                neighbors.push_back({cell.row, cell.col - 1});
                break;
            case 'R':
                neighbors.push_back({cell.row, cell.col + 1});
                break;
            case 'U':
                neighbors.push_back({cell.row - 1, cell.col});
                break;
            case 'D':
                neighbors.push_back({cell.row + 1, cell.col});
                break;
        }
    }

    neighbors.erase(std::remove_if(neighbors.begin(), neighbors.end(),
        [&](const Cell& n) {
            return (n.row < 0 || n.row >= rows || n.col < 0 || n.col >= columns);
        }), neighbors.end());

    return neighbors;
}

double platFinding::CalculateHeuristic(Cell cell, Cell goal) {
    return std::sqrt(std::pow(cell.row - goal.row, 2) + std::pow(cell.col - goal.col, 2));
}

std::vector<Cell> platFinding::FindPath(Cell start, Cell goal) {
    std::priority_queue<CostCellPair> openSet;

    std::map<Cell, Cell, CellCompare> cameFrom;
    std::map<Cell, double, CellCompare> gScore;
    std::map<Cell, double, CellCompare> fScore;
    std::set<Cell, CellCompare> closedSet;

    openSet.push(CostCellPair(0, start));
    gScore[start] = 0;
    fScore[start] = CalculateHeuristic(start, goal);

    while (!openSet.empty()) {
        CostCellPair currentPair = openSet.top();
        openSet.pop();
        Cell current = currentPair.cell;

        if (current.row == goal.row && current.col == goal.col) {
            return ReconstructPath(cameFrom, current);
        }

        closedSet.insert(current);

        for (const Cell& neighbor : GetNeighbors(current)) {
            if (closedSet.count(neighbor) > 0)
                continue;

            double tentativeGScore = gScore[current] + CalculateHeuristic(current, neighbor);

            if (gScore.find(neighbor) == gScore.end() || tentativeGScore < gScore[neighbor]) {
                cameFrom[neighbor] = current;
                gScore[neighbor] = tentativeGScore;
                fScore[neighbor] = gScore[neighbor] + CalculateHeuristic(neighbor, goal);

                openSet.push(CostCellPair(fScore[neighbor], neighbor));
            }
        }
    }

    return std::vector<Cell>(); // No path found
}

std::vector<Cell> platFinding::ReconstructPath(std::map<Cell, Cell, CellCompare> cameFrom, Cell current) {
    std::vector<Cell> path;
    path.push_back(current);

    while (cameFrom.find(current) != cameFrom.end()) {
        current = cameFrom[current];
        path.push_back(current);
    }

    std::reverse(path.begin(), path.end());
    return path;
}
