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


//subscribing to darknet_ros nodes
// #include "darknet_ros_msgs/BoundingBoxes.h"


// class Sensor {
//     public:
//     Sensor();
//     void simulateEnvironments();
//     void detectObject();
//     void detectQRCode();
//     // void boundingBoxCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);
// }

Sensor::Sensor() {
    // Initialize ROS environment
    // rosEnv.simulate();
    // ros::NodeHandle nh;
    
    
    // ros::Subscriber sub = nh.subscribe("darknet_ros/bounding_boxes", 1000, &Sensor::boundingBoxCallback, this);
    // detectShelf(rosNode.returnPointCloud());
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

void Sensor::detectShelf(std::vector<geometry_msgs::Point32> pcl_points){
    
    if (pcl_points.size() > 0){
        std::cout << "true" << std::endl;
    }else{
        std::cout << "false" << std::endl;
    }
        // ROS_INFO_STREAM(pcl_points.size());
    
}

void Sensor::detectQRCode(sensor_msgs::Image image_) {
    // Camera QR code detection logic
    std::cout << "Detecting QR codes using camera..." << std::endl;
    // Add camera QR code detection logic here
    ROS_INFO_STREAM(image_);
    
}

// void Sensor::boundingBoxCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg) {
//     for (const auto& box : msg->bounding_boxes) {
//         if box.Class == "person" {
//             std::cout << "Human Detected! Shutting Down" << std::endl;
//             //add shutdown logic
//         }
//     }
// }

// int main(int argc, char **argv) {
//     ros::init(argc, argv, "object_detection_node");
//     Sensor sensor;
//     ros::spin();
//     return 0;
// }
