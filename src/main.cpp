
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

int main(int argc, char **argv) {
    ros::init(argc, argv, "my_robot_node");
    ros::NodeHandle nh;
    // Controller controller = Controller();
    ROSNode rosNode(nh);
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
