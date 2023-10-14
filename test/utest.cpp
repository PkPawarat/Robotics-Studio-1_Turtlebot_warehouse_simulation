#include <gtest/gtest.h>
#include <climits>
#include <vector>

#include <ros/package.h> //This tool allows to identify the path of the package on your system
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "ros/ros.h"

#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include "tf/transform_datatypes.h" //To use getYaw function from the quaternion of orientation

#include <gtest/gtest.h>

#include "/home/pawarat/catkin_ws/src/Robotics-Studio-1/src/ControllerInterface/library/pathfinding.h"
#include "/home/pawarat/catkin_ws/src/Robotics-Studio-1/src/ControllerInterface/library/controllerinterface.h"
#include "/home/pawarat/catkin_ws/src/Robotics-Studio-1/src/ControllerInterface/library/pathfinding.h"
#include "/home/pawarat/catkin_ws/src/Robotics-Studio-1/src/ControllerInterface/library/ROSNode.h"
#include "/home/pawarat/catkin_ws/src/Robotics-Studio-1/src/ControllerInterface/library/sensor.h"
#include "/home/pawarat/catkin_ws/src/Robotics-Studio-1/src/MissionInterface/library/missionInterface.h"
#include "/home/pawarat/catkin_ws/src/Robotics-Studio-1/src/MissionInterface/library/mission.h"


// Test case for finding a path
TEST(PathPlanning, FindPath) {
  // Define your nodes and edges
  std::cout << "STARTTTTTTTTTTTTTTTTTTTTTT:" << std::endl;
  PathPlanning pathFinder;

  // Add nodes and edges to the path planner
  std::vector<Node> nodes;/* Initialize your nodes */
  // Add 20 nodes with X and Y coordinates
  nodes.push_back({0.0, 0.0});
  nodes.push_back({1.0, 1.0});
  nodes.push_back({2.0, 2.0});
  nodes.push_back({3.0, 3.0});
  nodes.push_back({4.0, 4.0});
  nodes.push_back({5.0, 5.0});
  nodes.push_back({6.0, 6.0});
  nodes.push_back({7.0, 7.0});
  nodes.push_back({8.0, 8.0});
  nodes.push_back({9.0, 9.0});
  nodes.push_back({10.0, 10.0});
  nodes.push_back({11.0, 11.0});
  nodes.push_back({12.0, 12.0});
  nodes.push_back({13.0, 13.0});
  nodes.push_back({14.0, 14.0});
  nodes.push_back({15.0, 15.0});
  nodes.push_back({16.0, 16.0});
  nodes.push_back({17.0, 17.0});
  nodes.push_back({18.0, 18.0});
  nodes.push_back({19.0, 19.0});

  // Add more nodes here as needed...

  pathFinder.AutumeticAddingEdge(nodes);

  // Prompt the user for start and goal nodes
  Node start, goal;
  std::cout << "Enter start node (X Y): ";
  std::cin >> start.X >> start.Y;
  std::cout << "Enter goal node (X Y): ";
  std::cin >> goal.X >> goal.Y;

  // Debug output to check input values
  std::cout << "Start node: (" << start.X << "," << start.Y << ")" << std::endl;
  std::cout << "Goal node: (" << goal.X << "," << goal.Y << ")" << std::endl;

  // Find the closest nodes to the specified coordinates
  start = pathFinder.FindClosestNode(nodes, start);
  goal = pathFinder.FindClosestNode(nodes, goal);

  // Debug output to check closest nodes
  std::cout << "Closest start node: (" << start.X << "," << start.Y << ")" << std::endl;
  std::cout << "Closest goal node: (" << goal.X << "," << goal.Y << ")" << std::endl;

  // Find the shortest path
  std::vector<Node> path = pathFinder.ShortestPath(start, goal);

  // Display the path
  if (!path.empty()) {
      std::cout << "Shortest path from (" << start.X << "," << start.Y << ") to (" << goal.X << "," << goal.Y << "):" << std::endl;
      for (const Node& node : path) {
          std::cout << "(" << node.X << "," << node.Y << ") ";
      }
      std::cout << std::endl;
  } else {
      std::cout << "No path found from (" << start.X << "," << start.Y << ") to (" << goal.X << "," << goal.Y << ")." << std::endl;
  }

  // Debug output to check the path
  std::cout << "Path size: " << path.size() << std::endl;

  // Verify that a path is found
  ASSERT_FALSE(path.empty());

  // You can add more specific assertions to verify the correctness of the path
  // For example, check if the path starts at the start node and ends at the goal node
  EXPECT_EQ(path.front(), start);
  EXPECT_EQ(path.back(), goal);
}

TEST(PathPlanning, FindPath2) {
  // Define your nodes and edges
  std::cout << "STARTTTTTTTTTTTTTTTTTTTTTT:" << std::endl;
  PathPlanning pathFinder;

  // Add nodes and edges to the path planner
  std::vector<Node> nodes;/* Initialize your nodes */
  // Add 20 nodes with X and Y coordinates
  nodes.push_back({0.0, 0.0});
  nodes.push_back({1.0, 1.0});
  nodes.push_back({2.0, 2.0});
  nodes.push_back({3.0, 3.0});
  nodes.push_back({4.0, 4.0});
  nodes.push_back({5.0, 5.0});
  nodes.push_back({6.0, 6.0});
  nodes.push_back({7.0, 7.0});
  nodes.push_back({8.0, 8.0});
  nodes.push_back({9.0, 9.0});
  nodes.push_back({10.0, 10.0});
  nodes.push_back({11.0, 11.0});
  nodes.push_back({12.0, 12.0});
  nodes.push_back({13.0, 13.0});
  nodes.push_back({14.0, 14.0});
  nodes.push_back({15.0, 15.0});
  nodes.push_back({16.0, 16.0});
  nodes.push_back({17.0, 17.0});
  nodes.push_back({18.0, 18.0});
  nodes.push_back({19.0, 19.0});

  // Add more nodes here as needed...

  pathFinder.AutumeticAddingEdge(nodes);

  // Prompt the user for start and goal nodes
  Node start = Node(1.2, 1);
  Node goal = Node(11, 11);

  // Debug output to check input values
  std::cout << "Start node: (" << start.X << "," << start.Y << ")" << std::endl;
  std::cout << "Goal node: (" << goal.X << "," << goal.Y << ")" << std::endl;

  // Find the closest nodes to the specified coordinates
  start = pathFinder.FindClosestNode(nodes, start);
  goal = pathFinder.FindClosestNode(nodes, goal);

  // Debug output to check closest nodes
  std::cout << "Closest start node: (" << start.X << "," << start.Y << ")" << std::endl;
  std::cout << "Closest goal node: (" << goal.X << "," << goal.Y << ")" << std::endl;

  // Find the shortest path
  std::vector<Node> path = pathFinder.ShortestPath(start, goal);

  // Display the path
  if (!path.empty()) {
      std::cout << "Shortest path from (" << start.X << "," << start.Y << ") to (" << goal.X << "," << goal.Y << "):" << std::endl;
      for (const Node& node : path) {
          std::cout << "(" << node.X << "," << node.Y << ") ";
      }
      std::cout << std::endl;
  } else {
      std::cout << "No path found from (" << start.X << "," << start.Y << ") to (" << goal.X << "," << goal.Y << ")." << std::endl;
  }

  // Debug output to check the path
  std::cout << "Path size: " << path.size() << std::endl;

  // Verify that a path is found
  ASSERT_FALSE(path.empty());

  // You can add more specific assertions to verify the correctness of the path
  // For example, check if the path starts at the start node and ends at the goal node
  EXPECT_EQ(path.size(), 11);
  EXPECT_EQ(path.front(), start);
  EXPECT_EQ(path.back(), goal);
}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
