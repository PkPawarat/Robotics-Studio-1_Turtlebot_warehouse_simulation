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
#include "std_msgs/Float64.h"

#include "geometry_msgs/PoseArray.h"
#include <thread>
ROSNode::ROSNode(ros::NodeHandle nh) : nh_(nh){
    odom = nh_.subscribe("/odom", 1, &ROSNode::odomCallBack, this);
    // camera = nh_.subscribe("/camera/rgb/image_raw", 1000, &ROSNode::cameraCallBack, this);
    // lidarSensor = nh_.subscribe("/sensor", 1000, &ROSNode::lidarCallBack, this);
    // pub_vel = nh_.advertise<std_msgs::Float64>("/cmd_vel", 3, false);

    pub_goal = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 3, false);
}

void ROSNode::simulate() {
    // Simulate the environment using ROS
    std::cout << "Simulating environment using ROS..." << std::endl;
    // Add ROS simulation logic here
}

void ROSNode::odomCallBack(const nav_msgs::OdometryConstPtr &msg){  //Type: nav_msgs/Odometry
    // robotMtx_.lock();
    bot_odom = *msg;

    nav_msgs::Odometry newODOM = bot_odom;
    newODOM.pose.pose.position.x += 0.5;
    newODOM.pose.pose.position.y += 0.5;

    geometry_msgs::PoseStamped goal;
    goal.pose.position = newODOM.pose.pose.position;
    goal.pose.orientation.w = 1;
    goal.header.frame_id = "map";

    sendGoal(goal);
    std::this_thread::sleep_for(std::chrono::seconds(10));
    // robotMtx_.unlock();
}
void ROSNode::cameraCallBack(const sensor_msgs::LaserScanConstPtr &msg){ // Type: sensor_msgs/Image
    robotMtx_.lock();
    camera_ = *msg;
    robotMtx_.unlock();
}
void ROSNode::lidarCallBack(const sensor_msgs::LaserScanConstPtr &msg){ //Type: sensor_msgs/LaserScan
    robotMtx_.lock();
    lidar_sensor = *msg;
    robotMtx_.unlock();
}
void ROSNode::sendCmd(int x, int y){
    
}

void ROSNode::sendGoal(geometry_msgs::PoseStamped goal){
    /////move_base_simple/goal geometry_msgs/PoseStamped
    pub_goal.publish(goal);
}
