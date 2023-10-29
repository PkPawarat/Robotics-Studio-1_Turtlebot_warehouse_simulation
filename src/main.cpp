
#include <iostream>
#include <vector>
#include "ControllerInterface/library/controller.h"
#include "ControllerInterface/library/controllerinterface.h"
#include "ControllerInterface/library/pathfinding.h"
#include "ControllerInterface/library/ROSNode.h"
#include "ControllerInterface/library/sensor.h"
#include "MissionInterface/library/missionInterface.h"
#include "MissionInterface/library/mission.h"

#include "ros/ros.h"
#include <thread>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// ros::Publisher initialPosePublisher;
// ros::Subscriber odom;

// void odomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg) {
//     // Create a message to publish to /initialpose
//     geometry_msgs::PoseWithCovarianceStamped initialPoseMsg;
    
//     // Extract relevant data from the odomMsg and set it in initialPoseMsg
//     initialPoseMsg.pose.pose = odomMsg->pose.pose;
//     std::string map = "map";
//     initialPoseMsg.header.frame_id = map;
//     // Publish the initial pose message
//     initialPosePublisher.publish(initialPoseMsg);

//     // Unsubscribe after receiving one message
//     ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", ros::Duration(1.0)); // Wait for one message
//     ros::shutdown();
// }

int main(int argc, char **argv) {
    ros::init(argc, argv, "my_robot_node");
    ros::NodeHandle nh;
    // Controller controller = Controller();
    ROSNode rosNode(nh);
    // Sensor sensor;
    ros::spin();
    ros::shutdown();
    // t.join();
    return 0;
}

// Test initial pose 
// int main(int argc, char** argv) {
//     // ros::init(argc, argv, "");
//     // ros::NodeHandle nh;
//     // // Create a publisher for the /initialpose topic
//     // // odom = nh.subscribe("/odom", 1, &odomCallback);
//     // // initialPosePublisher = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
//     // // Spin to process incoming messages
//     // ros::spin();
//     // ros::shutdown();
//     return 0;
// }

// int main() {
    
//     // // Create an instance of the PathPlanning class
//     // PathPlanning pathFinder;

//     // // Add nodes and edges to the path planner
//     // std::vector<Node> nodes;/* Initialize your nodes */
//     // // Add 20 nodes with X and Y coordinates
//     // nodes.push_back({0.0, 0.0});
//     // nodes.push_back({1.0, 1.0});
//     // nodes.push_back({2.0, 2.0});
//     // nodes.push_back({3.0, 3.0});
//     // nodes.push_back({4.0, 4.0});
//     // nodes.push_back({5.0, 5.0});
//     // nodes.push_back({6.0, 6.0});
//     // nodes.push_back({7.0, 7.0});
//     // nodes.push_back({8.0, 8.0});
//     // nodes.push_back({9.0, 9.0});
//     // nodes.push_back({10.0, 10.0});
//     // nodes.push_back({11.0, 11.0});
//     // nodes.push_back({12.0, 12.0});
//     // nodes.push_back({13.0, 13.0});
//     // nodes.push_back({14.0, 14.0});
//     // nodes.push_back({15.0, 15.0});
//     // nodes.push_back({16.0, 16.0});
//     // nodes.push_back({17.0, 17.0});
//     // nodes.push_back({18.0, 18.0});
//     // nodes.push_back({19.0, 19.0});
//     // pathFinder.AutumeticAddingEdge(nodes);

//     // while (true) {
//     //     // Prompt the user for start and goal nodes
//     //     Node start, goal;
//     //     std::cout << "Enter start node (X Y): ";
//     //     std::cin >> start.X >> start.Y;
//     //     std::cout << "Enter goal node (X Y): ";
//     //     std::cin >> goal.X >> goal.Y;

//     //     // Find the closest nodes to the specified coordinates
//     //     start = pathFinder.FindClosestNode(nodes, start);
//     //     goal = pathFinder.FindClosestNode(nodes, goal);

//     //     // Find the shortest path
//     //     std::vector<Node> path = pathFinder.ShortestPath(start, goal);

//     //     // Display the path
//     //     if (!path.empty()) {
//     //         std::cout << "Shortest path from (" << start.X << "," << start.Y << ") to (" << goal.X << "," << goal.Y << "):" << std::endl;
//     //         for (const Node& node : path) {
//     //             std::cout << "(" << node.X << "," << node.Y << ") ";
//     //         }
//     //         std::cout << std::endl;
//     //     } else {
//     //         std::cout << "No path found from (" << start.X << "," << start.Y << ") to (" << goal.X << "," << goal.Y << ")." << std::endl;
//     //     }

//     //     // Ask the user if they want to continue or exit
//     //     std::cout << "Do you want to find another path? (y/n): ";
//     //     char choice;
//     //     std::cin >> choice;

//     //     if (choice != 'y' && choice != 'Y') {
//     //         break; // Exit the loop if the user chooses not to continue
//     //     }
//     // }

//     return 0;
// }
