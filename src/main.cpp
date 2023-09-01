#include <iostream>
#include "ControllerInterface/library/controller.h"
#include "ControllerInterface/library/controllerinterface.h"
#include "ControllerInterface/library/pathfinding.h"
#include "ControllerInterface/library/ROSEnvironment.h"
#include "ControllerInterface/library/sensor.h"
#include "MissionInterface/library/missionInterface.h"
#include "MissionInterface/library/mission.h"

int main() {
    // int** grid = new int*[10];
    // for (int i = 0; i < 10; ++i) {
    //     grid[i] = new int[10];
    //     for (int j = 0; j < 10; ++j) {
    //         // Initialize grid elements based on input
    //         std::cin >> grid[i][j];
    //     }
    // }

    // AStar aStar(grid, 10, 10);
    // std::vector<Cell> path = aStar.FindPath({0, 0}, {9, 9});

    // if (!path.empty()) {
    //     std::cout << "Path found." << std::endl;

    //     // Create a copy of the grid to mark the path
    //     // ...

    //     // Print the path-marked grid
    //     // ...
    // } else {
    //     std::cout << "No path found." << std::endl;
    // }

    // // Clean up dynamically allocated memory for grid
    // for (int i = 0; i < 10; ++i) {
    //     delete[] grid[i];
    // }
    // delete[] grid;

    return 0;
}
