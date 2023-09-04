#include "library/ROSNode.h" 
#include <iostream>

// Keep only the headers needed
#include <vector>
#include "ros/ros.h"
#include <atomic>
#include <mutex>

#include "visualization_msgs/MarkerArray.h"
#include "std_srvs/SetBool.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/LaserScan.h"

#include "geometry_msgs/PoseArray.h"

ROSNode::ROSNode(ros::NodeHandle nh) : nh_(nh){
    odom = nh_.subscribe("/odom", 1000, &ROSNode::odomCallBack, this);
    pub_vel = nh_.advertise("/cmd_vel", 3, false);
}

void ROSNode::simulate() {
    // Simulate the environment using ROS
    std::cout << "Simulating environment using ROS..." << std::endl;
    // Add ROS simulation logic here
}

void ROSNode::odomCallBack(const nav_msgs::OdometryConstPtr &msg){
    robotOdomMtx_.lock();
    bot_odom = *msg;
    robotOdomMtx_.unlock();

}

void ROSNode::sendCmd(int x, int y){

}
