#ifndef ROSNODE_H
#define ROSNODE_H

#include <iostream>
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
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"

/**
 * @class ROSEnvironment
 * @brief A class for communicating with the ROS environment.
 */
class ROSNode
{
public:
    /**
     * @brief Constructor for the ROSEnvironment class.
     * @param nh The ROS NodeHandle.
     */
    ROSNode(ros::NodeHandle nh);

    /**
     * @brief Simulate the environment.
     */
    void simulate();

    /**
     * @brief Callback function for receiving odometry data.
     * @param msg The received Odometry message.
     */
    void odomCallBack(const nav_msgs::OdometryConstPtr &msg);

    /**
     * @brief Callback function for receiving laser scan data.
     * @param msg The received LaserScan message.
     */
    void laserScanCallBack(const sensor_msgs::LaserScanConstPtr &msg);

    /**
     * @brief Callback function for receiving camera image data.
     * @param msg The received Image message.
     */
    void cameraCallBack(const sensor_msgs::ImageConstPtr &msg);

    /**
     * @brief Get the latest odometry data.
     * @return The latest Odometry message.
     */
    nav_msgs::Odometry returnOdom();

    sensor_msgs::Image returnImage();

    /**
     * @brief Send a command to control the robot's movement.
     * @param linear_x The linear velocity in the x-axis.
     * @param linear_y The linear velocity in the y-axis.
     * @param linear_z The linear velocity in the z-axis.
     * @param angular_x The angular velocity in the x-axis.
     * @param angular_y The angular velocity in the y-axis.
     * @param angular_z The angular velocity in the z-axis.
     */
    void sendCmd(int linear_x, int linear_y, int linear_z, int angular_x, int angular_y, int angular_z);

public:
    ros::NodeHandle nh_;

    ros::Subscriber odom;
    ros::Subscriber laser_scan;
    ros::Subscriber camera;

    ros::Publisher pub_vel;
    geometry_msgs::Twist bot_vel;
    nav_msgs::Odometry bot_odom;
    sensor_msgs::LaserScan bot_laser_scan;
    sensor_msgs::Image image_;

    std::mutex robotMtx_;
    // std::mutex laserScanMtx_;
};

#endif
