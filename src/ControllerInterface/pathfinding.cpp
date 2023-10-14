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

#include <cmath>
#include <algorithm>

bool Node::operator==(const Node& other) const {
    return X == other.X && Y == other.Y;
}

bool Node::operator<(const Node& other) const {
    if (X != other.X) return X < other.X;
    return Y < other.Y;
}

size_t std::hash<Node>::operator()(const Node& node) const {
    return hash<double>()(node.X) ^ hash<double>()(node.Y);
}

void PathPlanning::AddEdge(const Node& from, const Node& to, double cost) {
    graph[from].emplace_back(to, cost);
}

void PathPlanning::AutumeticAddingEdge(const vector<Node>& AllNode) {
    for (int i = 0; i < AllNode.size(); i++) {
        for (int j = i + 1; j < AllNode.size(); j++) {
            if (i != j
                && abs(AllNode[i].X - AllNode[j].X) <= MaxDistanceNode
                && abs(AllNode[i].Y - AllNode[j].Y) <= MaxDistanceNode)
            {
                AddEdge(AllNode[i], AllNode[j], 1.0);
                AddEdge(AllNode[j], AllNode[i], 1.0);
            }
        }
    }
}

vector<Node> PathPlanning::ShortestPath(const Node& start, const Node& destination) {
    unordered_map<Node, double> distance;
    unordered_map<Node, Node> previous;
    set<Node> visited;
    priority_queue<pair<double, Node>, vector<pair<double, Node>>, greater<pair<double, Node>>> queue;

    for (const auto& pair : graph) {
        distance[pair.first] = numeric_limits<double>::infinity();
    }

    distance[start] = 0.0;
    queue.emplace(0.0, start);

    while (!queue.empty()) {
        double current_distance = queue.top().first;
        Node current = queue.top().second;
        queue.pop();

        if (visited.find(current) != visited.end()) 
            continue;
        visited.insert(current);

        if (current == destination) 
            break;

        if (graph.find(current) == graph.end()) 
            continue;

        for (const auto& neighbor_weight_pair : graph[current]) {
            Node neighbor = neighbor_weight_pair.first;
            double weight = neighbor_weight_pair.second;
            double altDistance = distance[current] + weight;
            if (distance.find(neighbor) == distance.end() || altDistance < distance[neighbor]) {
                distance[neighbor] = altDistance;
                previous[neighbor] = current;
                queue.emplace(altDistance, neighbor);
            }
        }
    }

    vector<Node> path;
    for (Node node = destination; ; node = previous[node]) {
        path.push_back(node);
        if (previous.find(node) == previous.end()) 
            break;
    }
    reverse(path.begin(), path.end());

    return path;
}

// Function to find the closest node to a given coordinate
Node PathPlanning::FindClosestNode(const std::vector<Node>& nodes, const Node& target) {
    double minDistance = std::numeric_limits<double>::max();
    Node closestNode;

    for (const Node& node : nodes) {
        double distance = std::sqrt(std::pow(node.X - target.X, 2) + std::pow(node.Y - target.Y, 2));
        if (distance < minDistance) {
            minDistance = distance;
            closestNode = node;
        }
    }

    return closestNode;
}
// ////////////////////////////////////////How to use this class 

// int main() {
//     // Create an instance of the PathPlanning class
//     PathPlanning pathFinder;

//     // Add nodes and edges to the path planner
//     std::vector<Node> nodes;/* Initialize your nodes */
//     // Add 20 nodes with X and Y coordinates
//     nodes.push_back({0.0, 0.0});
//     nodes.push_back({1.0, 1.0});
//     nodes.push_back({2.0, 2.0});
//     nodes.push_back({3.0, 3.0});
//     nodes.push_back({4.0, 4.0});
//     nodes.push_back({5.0, 5.0});
//     nodes.push_back({6.0, 6.0});
//     nodes.push_back({7.0, 7.0});
//     nodes.push_back({8.0, 8.0});
//     nodes.push_back({9.0, 9.0});
//     nodes.push_back({10.0, 10.0});
//     nodes.push_back({11.0, 11.0});
//     nodes.push_back({12.0, 12.0});
//     nodes.push_back({13.0, 13.0});
//     nodes.push_back({14.0, 14.0});
//     nodes.push_back({15.0, 15.0});
//     nodes.push_back({16.0, 16.0});
//     nodes.push_back({17.0, 17.0});
//     nodes.push_back({18.0, 18.0});
//     nodes.push_back({19.0, 19.0});
//     pathFinder.AutumeticAddingEdge(nodes);

//     while (true) {
//         // Prompt the user for start and goal nodes
//         Node start, goal;
//         std::cout << "Enter start node (X Y): ";
//         std::cin >> start.X >> start.Y;
//         std::cout << "Enter goal node (X Y): ";
//         std::cin >> goal.X >> goal.Y;

//         // Find the closest nodes to the specified coordinates
//         start = pathFinder.FindClosestNode(nodes, start);
//         goal = pathFinder.FindClosestNode(nodes, goal);

//         // Find the shortest path
//         std::vector<Node> path = pathFinder.ShortestPath(start, goal);

//         // Display the path
//         if (!path.empty()) {
//             std::cout << "Shortest path from (" << start.X << "," << start.Y << ") to (" << goal.X << "," << goal.Y << "):" << std::endl;
//             for (const Node& node : path) {
//                 std::cout << "(" << node.X << "," << node.Y << ") ";
//             }
//             std::cout << std::endl;
//         } else {
//             std::cout << "No path found from (" << start.X << "," << start.Y << ") to (" << goal.X << "," << goal.Y << ")." << std::endl;
//         }

//         // Ask the user if they want to continue or exit
//         std::cout << "Do you want to find another path? (y/n): ";
//         char choice;
//         std::cin >> choice;

//         if (choice != 'y' && choice != 'Y') {
//             break; // Exit the loop if the user chooses not to continue
//         }
//     }

//     return 0;
// }
