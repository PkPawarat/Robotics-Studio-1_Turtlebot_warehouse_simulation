#pragma once
#include "ROSNode.h"

// Keep only the headers needed
#include <vector>
#include <atomic>
#include <mutex>

#include "visualization_msgs/MarkerArray.h"
#include "std_srvs/SetBool.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseArray.h"
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
    void detectShelf(sensor_msgs::PointCloud2 point_cloud);
    void detectQRCode(sensor_msgs::Image image_);

public:
// bot_laser_scan from ROSNode
    sensor_msgs::Image raw_image_;

};
