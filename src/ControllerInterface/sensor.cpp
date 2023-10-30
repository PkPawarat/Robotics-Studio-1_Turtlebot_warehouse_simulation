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
    // shelf height is above 0.4 (may need to adjust as i believe it might be higher)

    float temp_x = 0;
    std::cout << pcl_points.size() << std::endl; // returns size of pcl_cloud
    std::cout << pcl_points.at(0) << std::endl; // returns first point x,y,z values
    std::cout << pcl_points.at(pcl_points.size()-1) << std::endl;
    std::vector<float> z_coords;
    float count = 0;
    float count_2 = 0;
    float temp_ = 0;
    
    for(int i = 0; i < pcl_points.size(); ++i){     //returns identified shelves STILL WORK IN PROGRESS
        if(pcl_points.at(i).y > 0 && pcl_points.at(i).z > 0.4){ // may need to adjust z value as the shelf might be higher
            // count_2++;
            // std::cout << pcl_points.at(i) << std::endl;
            
            if(abs(pcl_points.at(i).y) - temp_ < 0.1){
                std::cout << "still shelf" << std::endl;
            }else{
                std::cout << "next shelf" << std::endl;
            }
            temp_ = pcl_points.at(i).y;
        }
    }
    // float count = 0;
    // for (int k = 0; k < z_coords.size(); ++k){
    //     if(z_coords.at(k) > 0){
    //         count++;
    //     }
    // }
    std::cout << count_2 << std::endl;
    std::cout << count << std::endl;
    // std::cout << z_coords.size() << std::endl;
    // float max_z = z_coords[0];
    // for(int j = 1; j < z_coords.size()-1; ++j){
    //     if(z_coords.at(j) < max_z){
    //         max_z = z_coords.at(j);
    //     }
    //     // std::cout << z_coords.at(j) << std::endl;
    // }

    // std::cout << z_coords.at(0) << std::endl;
    // std::cout << max_z << std::endl;
        // ROS_INFO_STREAM(pcl_points.size());
    
}

void Sensor::detectQRCode(sensor_msgs::Image image_) {
    // Camera QR code detection logic
    std::cout << "Detecting QR codes using camera..." << std::endl;
    // Add camera QR code detection logic here
    // ROS_INFO_STREAM(image_);
    
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
