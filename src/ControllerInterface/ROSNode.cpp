
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

ROSNode::ROSNode(ros::NodeHandle nh) : nh_(nh)
{
    odom = nh_.subscribe("/odom", 1000, &ROSNode::odomCallBack, this);
    laser_scan = nh_.subscribe("/scan", 1000, &ROSNode::laserScanCallBack, this);
    camera = nh_.subscribe("/usb_cam/image_raw", 1, &ROSNode::cameraCallBack, this);

    pub_vel = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 3, false);
}

void ROSNode::simulate()
{
    // Simulate the environment using ROS
    std::cout << "Simulating environment using ROS..." << std::endl;
    // Add ROS simulation logic here
}

void ROSNode::odomCallBack(const nav_msgs::OdometryConstPtr &msg)
{
    robotMtx_.lock();
    bot_odom = *msg;
    robotMtx_.unlock();
}

void ROSNode::laserScanCallBack(const sensor_msgs::LaserScanConstPtr &msg)
{
    robotMtx_.lock();
    bot_laser_scan = *msg;
    robotMtx_.unlock();
}

void ROSNode::cameraCallBack(const sensor_msgs::ImageConstPtr &msg)
{ // Type: sensor_msgs/Image
    robotMtx_.lock();
    image_ = *msg;
    robotMtx_.unlock();
}

nav_msgs::Odometry ROSNode::returnOdom()
{
    return bot_odom;
}

sensor_msgs::Image ROSNode::returnImage()
{
    return image_;
}

void ROSNode::sendCmd(int linear_x, int linear_y, int linear_z, int angular_x, int angular_y, int angular_z)
{
    bot_vel.linear.x = linear_x;
    bot_vel.linear.y = linear_y;
    bot_vel.linear.z = linear_z;
    bot_vel.angular.x = angular_x;
    bot_vel.angular.y = angular_y;
    bot_vel.angular.z = angular_z;
    pub_vel.publish(bot_vel);
}
