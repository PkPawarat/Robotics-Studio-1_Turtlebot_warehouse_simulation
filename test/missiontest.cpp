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
#include "ControllerInterface/library/controller.h"
#include "ControllerInterface/library/pathfinding.h"
#include "ControllerInterface/library/ROSNode.h"
#include "ControllerInterface/library/sensor.h"
#include "MissionInterface/library/missionInterface.h"
#include "MissionInterface/library/mission.h"

#include <random>


// Test case for finding a path
TEST(Controller, TestController) {
    int argc = 0; // Initialize argc and argv for ROS
    char** argv = nullptr;

    ros::init(argc, argv, "Test node");
    ros::NodeHandle nh;
    ROSNode rosNode(nh);

    Controller* controller_= new Controller(&rosNode);
    PathPlanning pathFinder;

    std::string path = ros::package::getPath("Robotics-Studio-1");
    path += "/test/";
    std::string file = path + "pointLocation.bag";

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

            geometry_msgs::Point p;
            p.x = x;
            p.y = y;

            points.push_back(p);

            Node node = {x, y};
            nodes.push_back(node);
        }
    }

    // for (int i = 0; i < nodes.size(); i++)
    // {
    //     geometry_msgs::Point p;
    //     p.x = nodes.at(i).X;
    //     p.y = nodes.at(i).Y;

    //     points.push_back(p);
    // }


    controller_->SetTargets(points);
    controller_->SetPathPlanning(pathFinder, nodes);
    EXPECT_EQ(controller_->CountTargets(), nodes.size());

    controller_->Execute();
    

}



int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
