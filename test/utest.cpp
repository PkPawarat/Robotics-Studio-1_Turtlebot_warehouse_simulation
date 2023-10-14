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

#include <random>


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

// Test case for finding a path
TEST(PathPlanning, FindPathWithROSBag) {
    int argc = 0; // Initialize argc and argv for ROS
    char** argv = nullptr;
    // ros::init(argc, argv, "your_node_name");
    // ros::NodeHandle nh;

    // Open the ROS bag containing geometry_msgs/PoseStamped messages
    rosbag::Bag bag;
    bag.open("/home/pawarat/catkin_ws/src/Robotics-Studio-1/logs/pointLocation.bag", rosbag::bagmode::Read);

    PathPlanning pathFinder;

    // Define nodes and edges as needed
    std::vector<Node> nodes;
    // Add nodes and edges here...

    // Load data from the ROS bag into pathFinder
    rosbag::View view(bag);
    for (rosbag::MessageInstance const& msg : view) {
        geometry_msgs::PoseStamped::ConstPtr pose_msg = msg.instantiate<geometry_msgs::PoseStamped>();
        if (pose_msg != nullptr) {
            // Extract the X and Y coordinates from the PoseStamped message
            double x = pose_msg->pose.position.x;
            double y = pose_msg->pose.position.y;

            // Use x and y to create nodes and add them to your path planning graph
            Node node = {x, y};
            nodes.push_back(node);
        }
    }
    pathFinder.DrawMap(nodes, 0.5);
    // Add nodes and edges to the path planner
    pathFinder.AutumeticAddingEdge(nodes);

    // Prompt the user for start and goal nodes or use predefined values
    Node start = nodes.front();
    Node goal = nodes[15];

    std::cout << "Start node: (" << start.X << "," << start.Y << ")" << std::endl;
    std::cout << "Goal node: (" << goal.X << "," << goal.Y << ")" << std::endl;

    // Modify or remove this part if you want to provide start and goal nodes differently
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
    std::cout << "\n";
    std::cout << "\n";
    pathFinder.DrawMapWithShortestPath(nodes, path, 0.5);

    // Verify that a path is found
    // ASSERT_FALSE(path.empty());
    // EXPECT_EQ(path.front(), start);
    // EXPECT_EQ(path.back(), goal);
    bag.close();
}


// Test case for finding a path
TEST(PathPlanning, FindPathWithROSBagWithMultiGoal) {
    int argc = 0; // Initialize argc and argv for ROS
    char** argv = nullptr;
    // ros::init(argc, argv, "your_node_name");
    // ros::NodeHandle nh;

    // Open the ROS bag containing geometry_msgs/PoseStamped messages
    rosbag::Bag bag;
    bag.open("/home/pawarat/catkin_ws/src/Robotics-Studio-1/logs/pointLocation.bag", rosbag::bagmode::Read);

    PathPlanning pathFinder;

    // Define nodes and edges as needed
    std::vector<Node> nodes;
    // Add nodes and edges here...

    // Load data from the ROS bag into pathFinder
    rosbag::View view(bag);
    for (rosbag::MessageInstance const& msg : view) {
        geometry_msgs::PoseStamped::ConstPtr pose_msg = msg.instantiate<geometry_msgs::PoseStamped>();
        if (pose_msg != nullptr) {
            // Extract the X and Y coordinates from the PoseStamped message
            double x = pose_msg->pose.position.x;
            double y = pose_msg->pose.position.y;

            // Use x and y to create nodes and add them to your path planning graph
            Node node = {x, y};
            nodes.push_back(node);
        }
    }
    pathFinder.DrawMap(nodes, 0.5);
    // Add nodes and edges to the path planner
    pathFinder.AutumeticAddingEdge(nodes);

    int count = 0; 
    int countError = 0; 
    // Create random device and generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distrib(0, nodes.size() - 1);

    while (true){

        // Generate a random start node
        Node start = nodes[distrib(gen)];

        Node goal;
        do {
            // Generate a random goal node
            goal = nodes[distrib(gen)];
        } while (goal.X == start.X && goal.Y == start.Y);  // Ensure the goal is not the same as the start

        std::cout << "Start node: (" << start.X << "," << start.Y << ")" << std::endl;
        std::cout << "Goal node: (" << goal.X << "," << goal.Y << ")" << std::endl;

        // Modify or remove this part if you want to provide start and goal nodes differently
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
            std::cout << "\n";
            std::cout << "\n";

        } else {
            std::cout << "No path found from (" << start.X << "," << start.Y << ") to (" << goal.X << "," << goal.Y << ")." << std::endl;
        }
        std::cout << "\n";
        std::cout << "\n";
        pathFinder.DrawMapWithShortestPath(nodes, path, 0.5);
        std::cout << "\n";
        std::cout << "\n";


        count ++;
        if (path.size() == 1) countError ++;
        if (count == 200)break;
    }

    // Verify that a path is found
    // ASSERT_FALSE(path.empty());
    // EXPECT_EQ(path.front(), start);
    // EXPECT_EQ(path.back(), goal);
    std::cout << "Count error: " << countError << std::endl;


    bag.close();
}



int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
