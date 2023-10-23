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
}

// void Sensor::simulateEnvironments(){
//     // get data from ROS environment and display them?
//     raw_image_ = rosEnv.returnImage();

// }

void Sensor::detectObject(sensor_msgs::LaserScan bot_laser_scan) {
    // LIDAR object detection logic
    std::cout << "Detecting objects using LIDAR..." << std::endl;
    // Add LIDAR object detection logic her
}

void Sensor::detectQRCode(sensor_msgs::Image image_) {
    // Camera QR code detection logic
    std::cout << "Detecting QR codes using camera..." << std::endl;
    // Add camera QR code detection logic here
    ROS_INFO_STREAM(image_);
    
}
