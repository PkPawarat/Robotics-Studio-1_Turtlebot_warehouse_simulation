#include "library/controller.h" 
#include "library/pathfinding.h" 
#include "library/ROSNode.h" 
#include <iostream>

// Keep only the headers needed
#include <vector>
// #include "pfms_types.h"
#include "ros/ros.h"
#include <atomic>
#include <mutex>
#include <Eigen/Dense>

#include "visualization_msgs/MarkerArray.h"
#include "std_srvs/SetBool.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseArray.h"

Controller::Controller(ROSNode* rn) : 
    targetDetected(false), 
    qrCodeDetected(false), 
    obstacleDetected(false), 
    batteryLevel(true) 
    {
        ROSNode_ = rn;
        Targets.clear();

        //geometry_msgs::Point goal1{0.5,-0.5}; //The position of one of the pods
        //Targets.push_back(goal1);  
    }


int Controller::CountTargets()
{
    return Targets.size();
}

void Controller::Execute() 
{
    //TODO -> Do some checks here to determine if there is a collision pending

    //Waypoints are worked out in the pathfinder functions 

    std::cout << "\n Waypoint count: " + Targets.size() << std::endl;

    int a = Targets.size();
    for (unsigned int i = 0; i < Targets.size(); i++)
    {
        nav_msgs::Odometry start = ROSNode_->bot_odom;
        geometry_msgs::Point goal = Targets.at(i);

        // Finding the shortest path 
        Node start_node = _pathPlanning.SetNodeFromOdom(start);
        Node goal_node = _pathPlanning.SetNodeFromPoint(goal);
        start_node = _pathPlanning.FindClosestNode(_node, start_node);
        goal_node = _pathPlanning.FindClosestNode(_node, goal_node);
        std::vector<Node> path = _pathPlanning.ShortestPath(start_node, goal_node);

        //Drive to each of the waypoints
        for (unsigned int k = 0; k < path.size(); k++) {
             geometry_msgs::Point steps;
             steps.x = path.at(k).X;
             steps.y = path.at(k).Y;
             //Controller::TurnTo(steps);

             //check for collision
             //std::cout << "Robot turned towards waypoint. Driving function started" << std::endl;

             Controller::DriveTo(steps);
        }


        //Drive to the final point to the goal
        Controller::DriveTo(goal);

        std::cout << "\n Robot reached target. Holding in place to pick up / drop off (emulate): " << std::endl;

        //Stop the robot for a few seconds in place
        std::this_thread::sleep_for(std::chrono::seconds(10));
    }

    std::cout << "\n Driving complete: " + Targets.size() << std::endl;
}

void Controller::StartExecute() {
    std::thread execThread(&Controller::ThreadedExecute, this);
    execThread.detach(); // Detaching the thread if we don't need to join it later
}

void Controller::ThreadedExecute() {
    // Wait until ROSNode_->startNode becomes true
    unsigned int iterator = 0;
    while (true) {
        {
            std::lock_guard<std::mutex> lock(mtx); // Lock to check the shared variable safely
            if (ROSNode_->startNode) {
                break; // Exit the loop if startNode is true
            }
        std::cout << "Failed attempt to run Controller::Execute thread. Iterations: " << ++iterator << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Sleep to prevent busy waiting
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Sleep to prevent busy waiting
    }
    std::cout << "Execute thread enabled after iterations: " << iterator << std::endl;

    Execute(); // Now startNode is true, proceed with Execute
}

/// @brief set the target to Targets global variable, only use the location of the tagets.
/// @param targets 
void Controller::SetTargets(std::vector<geometry_msgs::Point> targets)
{
    Targets = targets;

    //Targets.resize(targets.size());
    //for(int i=0; i<targets.size(); i++){
       // Targets[i].location = targets[i];
    //}//
}


/// @brief set the target to Targets global variable, only use the location of the tagets.
/// @param targets 
void Controller::SetPathPlanning(PathPlanning pathPlanning, std::vector<Node> node)
{
    _pathPlanning = pathPlanning;
    _node = node;
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
void Controller::DriveTo(geometry_msgs::Point location)
 {
    std::cout << "Driving to x:" << location.x << ", y: " << location.y << "..." << std::endl;
    // return;
    // Add driving logic here
    double tolerance_ = 0.4;


    nav_msgs::Odometry odom = ROSNode_->returnOdom(); //Gathered from ROSNode

    //Turn the location into a pose

    geometry_msgs::Pose GoalPose; 
    GoalPose.position = location;
    GoalPose.orientation = odom.pose.pose.orientation; //Robots current direction facing is the orientation

    ROSNode_->sendGoal(GoalPose);

    //Hold in this function until the goal is reached
    double hyp = 9999;
    int iterator = 0;

    while (true)
    {
        nav_msgs::Odometry odom = ROSNode_->returnOdom(); //Gathered from ROSNode

        double current_x = odom.pose.pose.position.x;
        double current_y = odom.pose.pose.position.y;

        double delta_x = location.x - current_x;
        double delta_y = location.y - current_y;

        double hyp = sqrt(delta_x * delta_x + delta_y * delta_y);
        
        // ROSNode_->sendGoal(GoalPose);
        double i = abs(hyp);
        if (abs(hyp) <= tolerance_){
            break;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << "Waiting to reach to x:" << location.x << ", y: " << location.y << ". Iterator: " << ++iterator << std::endl;
    }
}

/// @brief This function will turn the robot to the point passed into it
/// @param location 
void Controller::TurnTo(geometry_msgs::Point target) 
{
    std::cout << "Initial rotation not functional: Target " << target << "..." << std::endl;
    return;

    nav_msgs::Odometry odom = ROSNode_->returnOdom(); //Gathered from ROSNode

    //Turn the location into a pose

    geometry_msgs::Pose GoalPose; 
    GoalPose.position = odom.pose.pose.position;
    //GoalPose.orientation = odom.pose.pose; //Robots current direction facing is the orientation

    ROSNode_->sendGoal(GoalPose);

    
    //Function helpers
    double angleTolerance = 0.1;
    double turningSpeed = 0.2; // Set the angular velocity to make the TurtleBot rotate. Adjust the value as needed

    //Use another function to calculate the turning required
    double delta_yaw = Controller::GetRotationTo(target);

    // If rotation is negative, swap direction
    if (delta_yaw < 0) turningSpeed = -turningSpeed;

    // Publish the command to start the rotation
    //ROSNode::sendCmd(0,0,0,0,0,turningSpeed);

    // Keep rotating until the angle difference is within tolerance
    while (abs(delta_yaw) > angleTolerance)
    {
        delta_yaw = Controller::GetRotationTo(target);
        
        // If rotation is negative, swap direction
        // if (delta_yaw > M_PI * 2) delta_yaw = delta_yaw - M_PI * 2;
        std::cout << "Rotation remaining: "  << delta_yaw << std::endl;
        
        // Pause briefly to control the loop rate
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

/// @brief Pass in a point on the map. This function will return (in radians) the rotation required to turn to the point.
double Controller::GetRotationTo(geometry_msgs::Point target)
{
    // Get the current pose of the robot
    nav_msgs::Odometry odom = ROSNode_->returnOdom(); //Gathered from ROSNode

    double current_x = odom.pose.pose.position.x;
    double current_y = odom.pose.pose.position.y;

    double delta_x = target.x - current_x;
    double delta_y = target.y - current_y;

    double targetYaw = atan2(delta_y, delta_x);

    Eigen::Quaterniond q;
    double angle_ = odom.twist.twist.angular.z;

    /*
    q.coeffs() << pose.Pose.Pose.Orientation.W, 
                pose.Pose.Pose.Orientation.X, pose.Pose.Pose.Orientation.Y,
                pose.Pose.Pose.Orientation.Z;
    
    Eigen::Vector3d euler_angles = q.toRotationMatrix().eulerAngles(0,1,2);
    */

    // Extract the current yaw angle
    //double CurrentRotation = euler_angles[2];
    double CurrentRotation = angle_;

    double delta_yaw = targetYaw - CurrentRotation;

    return delta_yaw;
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
