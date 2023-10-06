
#include <iostream>
#include <vector>
#include "ControllerInterface/library/controller.h"
#include "ControllerInterface/library/controllerinterface.h"
#include "ControllerInterface/library/pathfinding.h"
#include "ControllerInterface/library/ROSNode.h"
#include "ControllerInterface/library/sensor.h"
#include "MissionInterface/library/missionInterface.h"
#include "MissionInterface/library/mission.h"

#include "ros/ros.h"
#include <thread>


#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

ros::Publisher initialPosePublisher;

void odomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg) {
    // Create a message to publish to /initialpose
    geometry_msgs::PoseWithCovarianceStamped initialPoseMsg;
    
    // Extract relevant data from the odomMsg and set it in initialPoseMsg
    initialPoseMsg.pose.pose = odomMsg->pose.pose;
    
    // Publish the initial pose message
    initialPosePublisher.publish(initialPoseMsg);

    // Unsubscribe after receiving one message
    ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", ros::Duration(1.0)); // Wait for one message
    ros::shutdown();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_to_initialpose");
    ros::NodeHandle nh;

int main(int argc, char **argv) {
    // Replace "your_excel_file.xlsx" with the path to your Excel file
    // std::string excelFilePath = "/home/pk/git/Robotics-Studio-1/ExcelFile/PlatLocation1.csv";
    // // /home/pk/git/Robotics-Studio-1/ExcelFile
    // // /home/pawarat/git/Robotics-Studio-1/ExcelFile

    // // Create an instance of the platFinding class
    // platFinding pathFinder(excelFilePath);

    // // Define the start and goal cells
    // Cell start = {0, 0};
    // Cell goal = {5, 5};

    // // Find the path from start to goal
    // std::vector<Cell> path = pathFinder.FindPath(start, goal);

    // // Print the path
    // std::cout << "Path from (" << start.row << "," << start.col << ") to (" << goal.row << "," << goal.col << "):" << std::endl;
    // for (const Cell& cell : path) {
    //     std::cout << "(" << cell.row << "," << cell.col << ") ";
    // }
    // std::cout << std::endl;

    // printGrid(pathFinder.grid);

    ros::init(argc, argv, "my_robot_node");

    ros::NodeHandle nh;

    // std::shared_ptr<Controller> controller(new Ackerman(nh));
    // std::thread t(&Controller::separateThread,controller);

    Controller controller = Controller();
    ROSNode rosNode(nh);


    ros::spin();
        nav_msgs::Odometry newODOM = rosNode.bot_odom;
        newODOM.pose.pose.position.x += 2.5;
        newODOM.pose.pose.position.y += 2.5;

        geometry_msgs::PoseStamped goal;
        goal.pose.position = newODOM.pose.pose.position;
        goal.pose.orientation.w = 1;
        goal.header.frame_id = "map";
        rosNode.sendGoal(goal);

    ros::shutdown();

    // t.join();
    return 0;
}
