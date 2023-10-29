
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

#include <ros/package.h> //This tool allows to identify the path of the package on your system
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "ros/ros.h"

#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include "tf/transform_datatypes.h" //To use getYaw function from the quaternion of orientation
#include <random>


int main(int argc, char **argv) {
    ros::init(argc, argv, "my_robot_node");
    ros::NodeHandle nh;
    // Controller controller = Controller();
    ROSNode rosNode(nh);
    // Sensor sensor;

    Controller* controller_= new Controller(&rosNode);
    PathPlanning pathFinder;

    std::string path = ros::package::getPath("Robotics-Studio-1");
    path += "/test/";
    std::string file = path + "pointLocation.bag";
    file = "/home/pk/catkin_ws/src/Robotics-Studio-1/logs/pointLocation.bag";
    // Open the ROS bag containing geometry_msgs/PoseStamped messages from the point location
    rosbag::Bag bag;
    bag.open(file, rosbag::bagmode::Read);
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


    // For the Shelves locations
    
    //Turn nodes into targets
    std::vector<geometry_msgs::Point> points;

    file = path + "pickupshelf.bag";
    file = "/home/pk/catkin_ws/src/Robotics-Studio-1/logs/pickupshelf.bag";

    // Open the ROS bag containing geometry_msgs/PoseStamped messages from the point location
    rosbag::Bag bag2;
    bag2.open(file, rosbag::bagmode::Read);
    // Define nodes and edges as needed
    std::vector<Node> nodes2;
    // Add nodes and edges here...
    // Load data from the ROS bag into pathFinder
    rosbag::View view2(bag2);
    for (rosbag::MessageInstance const& msg : view2) {
        geometry_msgs::PoseStamped::ConstPtr pose_msg = msg.instantiate<geometry_msgs::PoseStamped>();
        if (pose_msg != nullptr) {
            // Extract the X and Y coordinates from the PoseStamped message
            double x = pose_msg->pose.position.x;
            double y = pose_msg->pose.position.y;

            geometry_msgs::Point p;
            p.x = x;
            p.y = y;

            points.push_back(p);

            Node node = {x, y};
            nodes2.push_back(node);
        }
    }

    controller_->SetTargets(points);
    controller_->SetPathPlanning(pathFinder, nodes);
    // Create a thread to run the Execute function
    std::thread execute_thread(&Controller::ThreadedExecute, controller_);
    // controller_->Execute();
    

    //Let's slow down this loop to 200ms (5Hz)
    // std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    ros::spin();
    ros::shutdown();
    execute_thread.join();
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
