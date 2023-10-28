#include "library/controller.h" 
#include "library/ROSNode.h" 
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
#include "ROSNode.h"

Controller::Controller() : 
    targetDetected(false), 
    qrCodeDetected(false), 
    obstacleDetected(false), 
    batteryLevel(true) 
    {
        Targets.clear();

        geometry_msgs::Point goal1{0.5,-0.5}; //The position of one of the pods
        Targets.push_back(goal1);  
    }


void Controller::Execute() {
    //TODO add more requirements

    //Do some checks here to determine if there is a collision pending



    //Work out the waypoints
    std::vector<geometry_msgs::Point> waypoints; 

//TEMP
    geometry_msgs::Point pt1{0.5, -2};
    geometry_msgs::Point pt2{3.5,-2.5};
    waypoints.push_back(pt1);
    waypoints.push_back(pt2);

    std::cout("Waypoint count: " + waypoints.size());

    for (unsigned int i = 0; i < waypoints.size(); i++)
    {
        var newPose = waypoints.at(i);

        TurnTo(newPose);

        //check for collision
        
        std::cout("Robot turned towards waypoint");

        DriveTo(newPose)
        sleep_for(2);
    }


/*
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
    */
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
    targetDetected = false; // Placeholder logic, replace with actual detection logic

    
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

/// @brief This function will mainly be used for moving the robot to target location using AStar methods from ROS
/// @param location 
void Controller::DriveTo(const std::string& location) {
    std::cout << "Driving to " << location << "..." << std::endl;
    // Add driving logic here

    



}

/// @brief This function will turn the robot to the point passed into it
/// @param location 
void Controller::TurnTo(geometry_msgs::Point target) {
    std::cout << "Turning to " << target << "..." << std::endl;
    // Add turning logic here

    //md_vel_pub = rospublisher('/cmd_vel', 'geometry_msgs/Twist');
    //pose_sub = rossubscriber('/odom', 'nav_msgs/Odometry');

    // Create a Twist message for the desired velocity command
    //cmd_msg = rosmessage(cmd_vel_pub);

    //send(cmd_vel_pub, cmd_msg);

    // Get the current pose of the robot
    //pose = receive(pose_sub);
    pose = bot_odom; //Gathered from ROSNode

    current_x = pose.Pose.Pose.Position.X
    current_y = pose.Pose.Pose.Position.Y

    delta_x = target.x - current_x
    delta_y = target.y - current_y

    targetYaw = atan2(delta_y, delta_x)

    currentOrientation = quat2eul([pose.Pose.Pose.Orientation.W, ...
        pose.Pose.Pose.Orientation.X, pose.Pose.Pose.Orientation.Y, ...
        pose.Pose.Pose.Orientation.Z]);

    // Extract the current yaw angle
    CurrentRotation = currentOrientation(1)

    delta_yaw = targetYaw - CurrentRotation
    delta_yaw_deg = rad2deg(delta_yaw)

    // Set the angular velocity to make the TurtleBot rotate
    turningSpeed = 0.2; // Adjust the value as needed
    if delta_yaw_deg < 0
        // If rotation is negative, swap direction
        turningSpeed = -turningSpeed;
    end

    // Publish the command to start the rotation
    //send(cmd_vel_pub, cmd_msg);
    ROSNode::sendCmd(0,0,0,0,0,turningSpeed)

    angleTolerance = 0.05;
    // Keep rotating until the angle difference is within tolerance
    while abs(delta_yaw_deg) > angleTolerance
        // Get the current orientation from the pose subscriber
        //pose = receive(pose_sub);
        pose = bot_odom; //Gathered from ROSNode

        currentOrientation = quat2eul([pose.Pose.Pose.Orientation.W, ...
            pose.Pose.Pose.Orientation.X, pose.Pose.Pose.Orientation.Y, ...
            pose.Pose.Pose.Orientation.Z]);
        CurrentRotation = currentOrientation(1); // Extract the yaw angle
        
        // Calculate the updated angle difference
        delta_yaw_deg = targetYaw - CurrentRotation;
        if delta_yaw_deg > pi * 2
            delta_yaw_deg = delta_yaw_deg - pi * 2;
        end
        disp(delta_yaw_deg);
        
        // Pause briefly to control the loop rate
        pause(0.02);
    end

    cmd_msg.Angular.Z = 0;    
    ROSNode::sendCmd(0,0,0,0,0,0)

    //send(cmd_vel_pub, cmd_msg);

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

void Controller::Stop() {
    std::cout << "Program Terminated" << std::endl;
    // Add drop logic here
}

void Controller::CheckObstacle() {
    if(LIDARreading = true){
        obstacleDetected = true;
    } else {
        obstacleDetected = false;
    }
}

void Controller::CheckBattery() {
    if(battery > 20){
        batteryLevel = true;
    } else {
        batteryLevel = false;
    }
}

void Controller::Charge() {}
    // Add logic here
void Controller::RePerentObject(){


}