#pragma once
// #include "ROSNode.h"

// Keep only the headers needed
#include <vector>
#include <atomic>
#include <mutex>
#include <math.h>
#include <limits>
#include <cmath>

#include "visualization_msgs/MarkerArray.h"
#include "std_srvs/SetBool.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseArray.h"
#include <iostream>
#include <thread>
#include <chrono>
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "std_msgs/Float64.h"
// #include "darknet_ros_msgs/BoundingBoxes.h"

/**
 * @file Sensor.h
 * @brief Definition of the Sensor class for Sensor simulation.
 * @author Pawarat Phatthanaphusakun
 * @version   1.01
 * @date      2023-08-28
 */

class Sensor {
private:
    

public:
    Sensor();
    // void simulateEnvironments();
    void detectObject(sensor_msgs::LaserScan bot_laser_scan);
    float detectShelf(std::vector<geometry_msgs::Point> laser_scan);
    void detectQRCode(sensor_msgs::Image image_);

public:
// bot_laser_scan from ROSNode
    sensor_msgs::Image raw_image_;

};
