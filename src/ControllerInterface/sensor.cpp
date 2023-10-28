#include "library/sensor.h" 

#include <iostream>

// Keep only the headers needed
#include <vector>
// #include "pfms_types.h"
#include "ros/ros.h"
#include <atomic>
#include <mutex>

#include "visualization_msgs/MarkerArray.h"
#include "std_srvs/SetBool.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseArray.h"

Sensor::Sensor() {
    // Initialize ROS environment
    // rosEnv.simulate();
    ros::NodeHandle nh;
    // ros::Subscriber sub = nh.subscribe("darknet_ros/bounding_boxes", 1000, &Sensor::boundingBoxCallback, this);
}

void Sensor::detectObject(sensor_msgs::LaserScan bot_laser_scan) {
    // LIDAR object detection logic
    std::cout << "Detecting objects using LIDAR..." << std::endl;
    // Add LIDAR object detection logic her
}

void Sensor::detectShelf(sensor_msgs::PointCloud2 point_cloud){
    
}

void Sensor::detectQRCode(sensor_msgs::Image image_) {
    // Camera QR code detection logic
    std::cout << "Detecting QR codes using camera..." << std::endl;
    // Add camera QR code detection logic here
    ROS_INFO_STREAM(image_);
    
}


