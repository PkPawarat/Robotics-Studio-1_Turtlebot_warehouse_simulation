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

float Sensor::detectShelf(std::vector<geometry_msgs::Point> laser_scan){
    std::vector<geometry_msgs::Point> filteredLaserScan_;
    
    std::vector<float> shelf_locations;
    
    int shelf = 0;

    for (int j = 0; j < laser_scan.size(); ++j){
        if(laser_scan.at(j).x  > 0 ){
            filteredLaserScan_.push_back(laser_scan.at(j));
        }
    }
    float temp_ = filteredLaserScan_.at(0).y;
    float first_y_point = filteredLaserScan_.at(0).y;
    for (int i = 1; i < filteredLaserScan_.size()-1; ++i){
        if ((abs(temp_)-abs(filteredLaserScan_.at(i).y)) < 1){
            // std::cout << "no" << std::endl;
        }else{
            // last_y_point = temp_;
            shelf_locations.push_back(first_y_point);
            shelf_locations.push_back(temp_);
            first_y_point = filteredLaserScan_.at(i).y;
            // std::cout << "yes" << std::endl;
            shelf++;
        }
        // std::cout << laser_scan.at(i) << std::endl;
        temp_ = filteredLaserScan_.at(i).y;
    }

    std::cout << shelf_locations.size() << std::endl;
    float max_y = shelf_locations.at(0);
    
    for(int k = 1; k < shelf_locations.size(); ++k){
        if(abs(shelf_locations.at(k)) > abs(max_y)){
            max_y = shelf_locations.at(k);
        }
    }
    float first_y_pair = max_y;
    
    for(int l = 0; l < shelf_locations.size(); ++l){
        std::cout << abs(shelf_locations.at(l)) << std::endl;
        if(abs(shelf_locations.at(l)) < abs(first_y_pair)){
            first_y_pair = shelf_locations.at(l);
        }
    }
    bool flag = false;
    if (first_y_pair > 0){
        bool flag = true;
    }
    float second_y_pair = max_y;
    for(int m = 0; m < shelf_locations.size(); ++m){
        std::cout << abs(shelf_locations.at(m)) << std::endl;
        if(flag = true){
            if(abs(shelf_locations.at(m)) < abs(second_y_pair) && shelf_locations.at(m) != first_y_pair && shelf_locations.at(m) < 0){
                second_y_pair = shelf_locations.at(m);
            }
        }else{
            if(abs(shelf_locations.at(m)) < abs(second_y_pair) && shelf_locations.at(m) != first_y_pair && shelf_locations.at(m) > 0){
                second_y_pair = shelf_locations.at(m);
            }
        }
    }
    std::cout << first_y_pair << ", " << second_y_pair << std::endl;

    return (first_y_pair+second_y_pair)/2;
    // shelf height is above 0.4 (may need to adjust as i believe it might be higher)

    // float temp_x = 0;
    // std::cout << pcl_points.size() << std::endl; // returns size of pcl_cloud
    // std::cout << pcl_points.at(0) << std::endl; // returns first point x,y,z values
    // std::cout << pcl_points.at(pcl_points.size()-1) << std::endl;
    // std::vector<float> z_coords;
    // float count = 0;
    // float count_2 = 0;
    // float temp_ = 0;
    
    // for(int i = 0; i < pcl_points.size(); ++i){     //returns identified shelves STILL WORK IN PROGRESS
    //     if(pcl_points.at(i).y > 0 && pcl_points.at(i).z > 0.8){ // may need to adjust z value as the shelf might be higher
    //         // count_2++;
    //         // std::cout << pcl_points.at(i) << std::endl;
            
    //         if((abs(pcl_points.at(i).y) - temp_) < 0.03){
    //             std::cout << "shelf 1" << std::endl;
    //         }else{
    //             std::cout << "shelf 2" << std::endl;
    //         }
    //         temp_ = pcl_points.at(i).y;
    //     }
    // }
    // float count = 0;
    // for (int k = 0; k < z_coords.size(); ++k){
    //     if(z_coords.at(k) > 0){
    //         count++;
    //     }
    // }
    // std::cout << count_2 << std::endl;
    // std::cout << count << std::endl;
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
