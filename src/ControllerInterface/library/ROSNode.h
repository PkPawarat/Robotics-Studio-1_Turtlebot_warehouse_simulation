#ifndef ROSNODE_H
#define ROSNODE_H

#include <iostream>
#include <vector>

#include "ros/ros.h"
#include "sensor.h"
#include <atomic>
#include <mutex>
#include <thread>
#include <chrono>
#include "visualization_msgs/MarkerArray.h"
#include "std_srvs/SetBool.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "std_msgs/Float64.h"



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

    void pointCloudCallBack(const sensor_msgs::PointCloud2ConstPtr &msg);

    /**
     * @brief Get the latest odometry data.
     * @return The latest Odometry message.
     */
    nav_msgs::Odometry returnOdom();

    /**
     * @brief Returns a LaserScan message.
     * @return sensor_msgs::LaserScan A LaserScan message.
     */
    std::vector<geometry_msgs::Point> returnLaserScan();

    /**
     * @brief Returns an Image message.
     * @return sensor_msgs::Image An Image message.
     */
    sensor_msgs::Image returnImage();

    std::vector<geometry_msgs::Point32> returnPointCloud();



/**
 * @brief Filters out sensor data beyond 1m. 
 * @return Filtered Sensor Data
*/
    std::vector<geometry_msgs::Point32>  returnReducedPointCloud();

    std::vector<geometry_msgs::Point32> filterRangeBounds();

    float calculateDistance(float x, float y, float z);

    geometry_msgs::Point polarToCart(unsigned int index);




    /**
     * @brief Send a command to control the robot's movement.
     * @param linear_x The linear velocity in the x-axis.
     * @param linear_y The linear velocity in the y-axis.
     * @param linear_z The linear velocity in the z-axis.
     * @param angular_x The angular velocity in the x-axis.
     * @param angular_y The angular velocity in the y-axis.
     * @param angular_z The angular velocity in the z-axis.
     */
    void sendCmd(double linear_x, double linear_y, double linear_z, double angular_x, double angular_y, double angular_z);

public:

    std::vector<geometry_msgs::Point32> pcl_points;
    // std::vector<geometry_msgs::Point> filteredLaserScan_;
    ros::NodeHandle nh_;

    ros::Subscriber odom;
    ros::Subscriber laser_scan;
    ros::Subscriber camera;
    ros::Subscriber camera_depth;

    ros::Publisher pub_vel;
    ros::Publisher pub_goal;
    geometry_msgs::Twist bot_vel;
    nav_msgs::Odometry bot_odom;
    sensor_msgs::LaserScan bot_laser_scan;
    sensor_msgs::LaserScan temp_scan;
    sensor_msgs::Image image_;
    sensor_msgs::PointCloud2 point_cloud;

    std::mutex robotMtx_;

    std::thread thread_;
    // std::mutex laserScanMtx_;
};

#endif
