#include "library/controller.h" 
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

Controller::Controller() : 
    targetDetected(false), 
    qrCodeDetected(false)
    {
    Targets.clear();

    }

void Controller::Execute() {
    //TODO add more requirements

    CheckTarget();
    if (targetDetected) {
        CheckQRCode();
        if (qrCodeDetected) {
            DriveTo(currentTarget);
            PickUpTarget();
            DriveTo("destination");
            DropTarget();
        } else {
            std::cout << "QR code not detected. Cannot proceed." << std::endl;
        }
    } else {
        std::cout << "No target detected." << std::endl;
    }
}
/// @brief set the target to Targets global variable, only use the location of the tagets.
/// @param targets 
void Controller::SetTargets(std::vector<geometry_msgs::Point> targets){
    Targets.resize(targets.size());
    for(int i=0; i<targets.size(); i++){
        Targets[i].location = targets[i];
    }
}

void Controller::CheckTarget() {
    targetDetected = true; // Placeholder logic, replace with actual detection logic

    // checking the current location of the robot with current location of target object if it in the correct location set targetDetected to true
}

/// @brief this function will assign target information to the robot to move to currrent target location which will need to CheckTarget() function if it in the correct location set targetDet.
/// @param target 
void Controller::AssignTarget(const std::string& target) {
    currentTarget = target;
    std::cout << "Assigned target: " << currentTarget << std::endl;
}

/// @brief This need to check from ROSNode that it have received a umage from camera sensor.
void Controller::CheckQRCode() {        // Parameter need to receive a image of from the camera sensor 
    qrCodeDetected = true; // Placeholder logic, replace with actual QR code detection logic
}

/// @brief This functio will mainly be used for moving the robot to target location using AStar methods from ROS
/// @param location 
void Controller::DriveTo(const std::string& location) {
    std::cout << "Driving to " << location << "..." << std::endl;
    // Add driving logic here
}

/// @brief This functio will pickup the target, specifically it will change the parent of the target object to robot and then move it to desired location
void Controller::PickUpTarget() {
    std::cout << "Picking up target: " << currentTarget << "..." << std::endl;
    // Add pick-up logic here
}

/// @brief This functio will drop the target, specifically it will change the parent of the target object to the sence 
void Controller::DropTarget() {
    std::cout << "Dropping target: " << currentTarget << "..." << std::endl;
    // Add drop logic here
}

void Controller::RePerentObject(){

}