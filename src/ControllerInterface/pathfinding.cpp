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

void PathPlanning::DrawMap(const std::vector<Node>& locations, double precision) {
    double minX = std::min_element(locations.begin(), locations.end(), [](const Node& a, const Node& b) { return a.X < b.X; })->X;
    double minY = std::min_element(locations.begin(), locations.end(), [](const Node& a, const Node& b) { return a.Y < b.Y; })->Y;
    double maxX = std::max_element(locations.begin(), locations.end(), [](const Node& a, const Node& b) { return a.X < b.X; })->X;
    double maxY = std::max_element(locations.begin(), locations.end(), [](const Node& a, const Node& b) { return a.Y < b.Y; })->Y;

    for (double y = minY; y <= maxY; y += precision) {
        for (double x = minX; x <= maxX; x += precision) {
            if (std::any_of(locations.begin(), locations.end(), [x, y, precision](const Node& n) { 
                return std::abs(n.X - x) < precision && std::abs(n.Y - y) < precision; 
            })) {
                std::cout << " * ";
            } else {
                std::cout << " . ";
            }
        }
        std::cout << std::endl;
    }
}

void PathPlanning::DrawMapWithShortestPath(const std::vector<Node>& locations, const std::vector<Node>& shortestPath, double precision) {
    double minX = std::min_element(locations.begin(), locations.end(), [](const Node& a, const Node& b) { return a.X < b.X; })->X;
    double minY = std::min_element(locations.begin(), locations.end(), [](const Node& a, const Node& b) { return a.Y < b.Y; })->Y;
    double maxX = std::max_element(locations.begin(), locations.end(), [](const Node& a, const Node& b) { return a.X < b.X; })->X;
    double maxY = std::max_element(locations.begin(), locations.end(), [](const Node& a, const Node& b) { return a.Y < b.Y; })->Y;

    int gridWidth = static_cast<int>((maxX - minX) / precision) + 1;
    int gridHeight = static_cast<int>((maxY - minY) / precision) + 1;
    
    std::vector<std::vector<std::string>> grid(gridHeight, std::vector<std::string>(gridWidth, " ' "));

    for (const Node& location : locations) {
        int x = static_cast<int>((location.X - minX) / precision);
        int y = static_cast<int>((location.Y - minY) / precision);
        grid[y][x] = " + ";
    }
    int i = 0;
    for (const Node& location : shortestPath) {
        int x = static_cast<int>((location.X - minX) / precision);
        int y = static_cast<int>((location.Y - minY) / precision);
        if (&location == &shortestPath.back()) {
            grid[y][x] = " X ";
        } else {
            grid[y][x] = " " + std::to_string(i) + " ";
        }
        i++;
    }

    for (int y = 0; y < gridHeight; y++) {
        for (int x = 0; x < gridWidth; x++) {
            std::cout << grid[y][x];
        }
        std::cout << std::endl;
    }


    std::cout << "STARTLOGGING" << std::endl;
    

    // For logging
    std::ofstream logFile("/home/connor/catkin_ws/src/Robotics-Studio-1/logs/path_log.txt", std::ios::app); // Open in append mode
    
    // Error checking for file opening
    if (!logFile.is_open()) {
        std::cerr << "Failed to open the log file." << std::endl;
        return;
    }
    // Log the path
    logFile << "Path from (" << shortestPath.front().X << "," << shortestPath.front().Y 
            << ") to (" << shortestPath.back().X << "," << shortestPath.back().Y << "):" << std::endl;

    logFile << "Shortest path " << std::endl;
    for (const Node& node : shortestPath) {
        logFile << "(" << node.X << "," << node.Y << ") ";
    }
    std::cout << std::endl;
    std::cout << "\n";

    for (int y = 0; y < gridHeight; y++) {
        for (int x = 0; x < gridWidth; x++) {
            logFile << grid[y][x];
        }
        logFile << std::endl;
    }


    logFile << std::endl; // Add a blank line to separate different paths in the log
    logFile.close(); // Always close the file when done
}

Node PathPlanning::SetNodeFromPoint(geometry_msgs::Point point){
    Node node;
    node.X = point.x;
    node.Y = point.y;
    return node;
}
Node PathPlanning::SetNodeFromOdom(nav_msgs::Odometry point){
    Node node;
    node.X = point.pose.pose.position.x;
    node.Y = point.pose.pose.position.y;
    return node;
}

geometry_msgs::Point PathPlanning::NodeToPoint(Node node){
    geometry_msgs::Point point;
    point.x = node.X;
    point.y = node.Y;
    return point;
}
