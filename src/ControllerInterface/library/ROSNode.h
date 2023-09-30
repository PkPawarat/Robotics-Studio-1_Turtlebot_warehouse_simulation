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

class ROSNode
{
    public:
        ROSNode(ros::NodeHandle nh){};

        void simulate();
        void odomCallBack(const nav_msgs::OdometryConstPtr &msg);
        void sendCmd(int x, int y);

    public:
        ros::NodeHandle nh_;
        ros::Subscriber odom;
        ros::Publisher pub_vel;

        nav_msgs::Odometry bot_odom;

};
#endif  // ROSNODE_H
