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

Controller::Controller() : targetDetected(false), qrCodeDetected(false), obstacleDetected(false), batteryLevel(true) {}

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

    CheckObstacle();
    if (obstacleDetected = true){
        Stop();
        std::cout << "Obstacle in Path. Program Treminated." << std::endl;
    } else {
        std::cout << "Path Clear" << std::endl;
    }

    CheckBattery();
    if(batteryLevel = true){
        std::cout << "Sufficient Battery Level." << std::endl;
    } else {
        Charge();
        std::cout << "Battery Low. Returning to Charging Bay" << std::endl;
    }
}

void Controller::CheckTarget() {
    targetDetected = true; // Placeholder logic, replace with actual detection logic
}

void Controller::AssignTarget(const std::string& target) {
    currentTarget = target;
    std::cout << "Assigned target: " << currentTarget << std::endl;
}

void Controller::CheckQRCode() {
    qrCodeDetected = true; // Placeholder logic, replace with actual QR code detection logic
}

void Controller::DriveTo(const std::string& location) {
    std::cout << "Driving to " << location << "..." << std::endl;
    // Add driving logic here
}

void Controller::PickUpTarget() {
    std::cout << "Picking up target: " << currentTarget << "..." << std::endl;
    // Add pick-up logic here
}

void Controller::DropTarget() {
    std::cout << "Dropping target: " << currentTarget << "..." << std::endl;
    // Add drop logic here
}

void Controller::Stop() {
    std::cout << "Program Terminated" << std::endl;
    // Add drop logic here
}

void Controller::CheckObstacle() {
    if(LIDARreading = true){
        obstacleDetected = true
    } else {
        obstacleDetected = false
    }
}

void Controller::CheckBattery() {
    if(battery > 20){
        batteryLevel = true
    } else {
        batteryLevel = false
    }
}

void Controller::Charge() {
    // Add logic here
}