
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

ros::Publisher initialPosePublisher;

void odomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg) {
    // Create a message to publish to /initialpose
    geometry_msgs::PoseWithCovarianceStamped initialPoseMsg;
    
    // Extract relevant data from the odomMsg and set it in initialPoseMsg
    initialPoseMsg.pose.pose = odomMsg->pose.pose;
    
    // Publish the initial pose message
    initialPosePublisher.publish(initialPoseMsg);

    // Unsubscribe after receiving one message
    ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", ros::Duration(1.0)); // Wait for one message
    ros::shutdown();
}

// int main(int argc, char **argv) {
//     ros::init(argc, argv, "my_robot_node");
//     ros::NodeHandle nh;
//     Controller controller = Controller();
//     ROSNode rosNode(nh);
//     ros::spin();
//     ros::shutdown();
//     // t.join();
//     return 0;
// }


int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_to_initialpose");
    ros::NodeHandle nh;
    // Create a publisher for the /initialpose topic
    initialPosePublisher = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
    // Spin to process incoming messages
    ros::spinOnce(); // Process messages once
    return 0;
}
