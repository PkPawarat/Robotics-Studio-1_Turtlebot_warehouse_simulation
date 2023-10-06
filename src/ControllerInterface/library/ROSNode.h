#ifndef ROSNODE_H
#define ROSNODE_H

#include <iostream>

// Keep only the headers needed
#include <vector>
#include "ros/ros.h"
#include <atomic>
#include <mutex>

#include "visualization_msgs/MarkerArray.h"
#include "std_srvs/SetBool.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseArray.h"

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseGoal.h>

class ROSNode
{
    public:
        ROSNode(ros::NodeHandle nh);

        void simulate();
        void odomCallBack(const nav_msgs::OdometryConstPtr &msg);
        void sendCmd(int x, int y);
        void cameraCallBack(const sensor_msgs::LaserScanConstPtr &msg);
        void lidarCallBack(const sensor_msgs::LaserScanConstPtr &msg);

        void sendGoal(geometry_msgs::PoseStamped goal);

    public:
        ros::NodeHandle nh_;
        ros::Subscriber odom;
        ros::Subscriber camera;
        ros::Subscriber lidarSensor;
        ros::Publisher pub_vel;
        ros::Publisher pub_goal;

        nav_msgs::Odometry bot_odom;
        sensor_msgs::LaserScan lidar_sensor;
        sensor_msgs::LaserScan camera_;     // need to change to something else

        std::mutex robotMtx_;
        
        move_base_msgs::MoveBaseGoal base_goal;
};
#endif  // ROSNODE_H
