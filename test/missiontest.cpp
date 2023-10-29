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

#include "ControllerInterface/library/pathfinding.h"
#include "ControllerInterface/library/controllerinterface.h"
#include "ControllerInterface/library/pathfinding.h"
#include "ControllerInterface/library/ROSNode.h"
#include "ControllerInterface/library/sensor.h"
#include "MissionInterface/library/missionInterface.h"
#include "MissionInterface/library/mission.h"

#include <random>

// Test case for finding a path
TEST(PathPlanning, FindPathWithROSBagWithMultiGoal) {
    int argc = 0; // Initialize argc and argv for ROS
    char** argv = nullptr;
    ros::init(argc, argv, "your_node_name");
    ros::NodeHandle nh;

    // // Open the ROS bag containing geometry_msgs/PoseStamped messages
    rosbag::Bag bag;
    bag.open("/home/connor/catkin_ws/src/Robotics-Studio-1/logs/pointLocation.bag", rosbag::bagmode::Read);

    PathPlanning pathFinder;

    // // Define nodes and edges as needed
    std::vector<Node> nodes;
    // // Add nodes and edges here...

    // // Load data from the ROS bag into pathFinder
    rosbag::View view(bag);
    // for (rosbag::MessageInstance const& msg : view) {
    //     geometry_msgs::PoseStamped::ConstPtr pose_msg = msg.instantiate<geometry_msgs::PoseStamped>();
    //     if (pose_msg != nullptr) {
    //         // Extract the X and Y coordinates from the PoseStamped message
    //         double x = pose_msg->pose.position.x;
    //         double y = pose_msg->pose.position.y;

    //         // Use x and y to create nodes and add them to your path planning graph
    //         Node node = {x, y};
    //         nodes.push_back(node);
    //     }
    // }
    // pathFinder.DrawMap(nodes, 0.5);
    // // Add nodes and edges to the path planner
    // pathFinder.AutumeticAddingEdge(nodes);

    // int count = 0; 
    // int countError = 0; 
    // // Create random device and generator
    // std::random_device rd;
    // std::mt19937 gen(rd());
    // std::uniform_int_distribution<> distrib(0, nodes.size() - 1);

    // while (true){

    //     // Generate a random start node
    //     Node start = nodes[distrib(gen)];

    //     Node goal;
    //     do {
    //         // Generate a random goal node
    //         goal = nodes[distrib(gen)];
    //     } while (goal.X == start.X && goal.Y == start.Y);  // Ensure the goal is not the same as the start

    //     std::cout << "Start node: (" << start.X << "," << start.Y << ")" << std::endl;
    //     std::cout << "Goal node: (" << goal.X << "," << goal.Y << ")" << std::endl;

    //     // Modify or remove this part if you want to provide start and goal nodes differently
    //     // Find the closest nodes to the specified coordinates
    //     start = pathFinder.FindClosestNode(nodes, start);
    //     goal = pathFinder.FindClosestNode(nodes, goal);

    //     // Debug output to check closest nodes
    //     std::cout << "Closest start node: (" << start.X << "," << start.Y << ")" << std::endl;
    //     std::cout << "Closest goal node: (" << goal.X << "," << goal.Y << ")" << std::endl;

    //     // Find the shortest path
    //     std::vector<Node> path = pathFinder.ShortestPath(start, goal);

    //     // Display the path
    //     if (!path.empty()) {
    //         std::cout << "Shortest path from (" << start.X << "," << start.Y << ") to (" << goal.X << "," << goal.Y << "):" << std::endl;
    //         for (const Node& node : path) {
    //             std::cout << "(" << node.X << "," << node.Y << ") ";
    //         }
    //         std::cout << std::endl;
    //         std::cout << "\n";
    //         std::cout << "\n";

    //     } else {
    //         std::cout << "No path found from (" << start.X << "," << start.Y << ") to (" << goal.X << "," << goal.Y << ")." << std::endl;
    //     }
    //     std::cout << "\n";
    //     std::cout << "\n";
    //     pathFinder.DrawMapWithShortestPath(nodes, path, 0.5);
    //     std::cout << "\n";
    //     std::cout << "\n";


    //     count ++;
    //     if (path.size() == 1) countError ++;
    //     if (count == 200)break;
    // }

    // // Verify that a path is found
    // // ASSERT_FALSE(path.empty());
    // // EXPECT_EQ(path.front(), start);
    // // EXPECT_EQ(path.back(), goal);
    // std::cout << "Count error: " << countError << std::endl;


    // bag.close();
}



int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
