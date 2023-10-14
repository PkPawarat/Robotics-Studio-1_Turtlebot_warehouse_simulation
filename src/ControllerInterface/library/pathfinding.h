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

#include <vector>
#include <unordered_map>
#include <queue>
#include <set>
#include <limits>

using namespace std;

class Node {
public:
    double X, Y;
    Node() : X(0), Y(0) {}
    Node(double x, double y) : X(x), Y(y) {}

    bool operator==(const Node& other) const;
    bool operator<(const Node& other) const;
};

namespace std {
    template <>
    struct hash<Node> {
        size_t operator()(const Node& node) const;
    };
}

class PathPlanning {
public:
    unordered_map<Node, vector<pair<Node, double>>> graph;
    double MaxDistanceNode = 1.0;

    void AddEdge(const Node& from, const Node& to, double cost);
    void AutumeticAddingEdge(const vector<Node>& AllNode);
    vector<Node> ShortestPath(const Node& start, const Node& destination);
    Node FindClosestNode(const std::vector<Node>& nodes, const Node& target);

};

#endif // PATHFINDING_H
