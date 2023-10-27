
#include "library/ROSNode.h"
// Keep only the headers needed
#include "ros/ros.h"


ROSNode::ROSNode(ros::NodeHandle nh) : nh_(nh){
    odom = nh_.subscribe("/odom", 1, &ROSNode::odomCallBack, this);
    laser_scan = nh_.subscribe("/scan", 1, &ROSNode::laserScanCallBack, this);
    camera = nh_.subscribe("/camera/rgb/image_raw", 1, &ROSNode::cameraCallBack, this);
    camera_depth = nh_.subscribe("/camera/depth/points", 1, &ROSNode::pointCloudCallBack, this);
// 
    pub_vel = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 3, false);
    pub_goal = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 3, false);
    // pub_goal = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 3, false);
    thread_ = std::thread(&ROSNode::simulate, this);
    thread_.detach();
    // camera = nh_.subscribe("/camera/rgb/image_raw", 1000, &ROSNode::cameraCallBack, this);
    // lidarSensor = nh_.subscribe("/sensor", 1000, &ROSNode::lidarCallBack, this);
    // pub_vel = nh_.advertise<std_msgs::Float64>("/cmd_vel", 3, false);

    
}

void ROSNode::simulate()
{
    // Simulate the environment using ROS
    std::cout << "Simulating environment using ROS..." << std::endl;
    // Add ROS simulation logic here
    std::this_thread::sleep_for(std::chrono::seconds(2));
    // ROS_INFO_STREAM(bot_odom);
    ROS_INFO_STREAM(pcl_points.size());
    // ROS_INFO_STREAM(point_cloud.width);
    
}

void ROSNode::odomCallBack(const nav_msgs::OdometryConstPtr &msg)
{
    robotMtx_.lock();
    bot_odom = *msg;
    robotMtx_.unlock();
}

void ROSNode::laserScanCallBack(const sensor_msgs::LaserScanConstPtr &msg)
{
    robotMtx_.lock();
    bot_laser_scan = *msg;
    robotMtx_.unlock();
}

void ROSNode::cameraCallBack(const sensor_msgs::ImageConstPtr &msg)
{ // Type: sensor_msgs/Image
    robotMtx_.lock();
    image_ = *msg;
    robotMtx_.unlock();
}

void ROSNode::pointCloudCallBack(const sensor_msgs::PointCloud2ConstPtr &msg)
{ // Type: sensor_msgs/Image
    robotMtx_.lock();

    sensor_msgs::PointCloud2 input_pointcloud = *msg;
    sensor_msgs::PointCloud out_pointcloud;
    sensor_msgs::convertPointCloud2ToPointCloud(input_pointcloud, out_pointcloud);

    std::vector<geometry_msgs::Point32> temp_;

    for(int i = 0 ; i < out_pointcloud.points.size(); ++i){
        geometry_msgs::Point32 point;

        //Dooo something here
        point.x = out_pointcloud.points[i].x;
        point.y = out_pointcloud.points[i].y;
        point.z = out_pointcloud.points[i].z;

        temp_.push_back(point);
    }
    pcl_points = temp_;
    robotMtx_.unlock();
}

nav_msgs::Odometry ROSNode::returnOdom()
{
    return bot_odom;
}

sensor_msgs::LaserScan ROSNode::returnLaserScan()
{
    return bot_laser_scan;
}

sensor_msgs::Image ROSNode::returnImage()
{
    return image_;
}


sensor_msgs::PointCloud2 ROSNode::returnPointCloud(){
    return point_cloud;
}


void ROSNode::sendCmd(double linear_x, double linear_y, double linear_z, double angular_x, double angular_y, double angular_z)
{
    bot_vel.linear.x = linear_x;
    bot_vel.linear.y = linear_y;
    bot_vel.linear.z = linear_z;
    bot_vel.angular.x = angular_x;
    bot_vel.angular.y = angular_y;
    bot_vel.angular.z = angular_z;
    pub_vel.publish(bot_vel);
}


// int main(int argc, char **argv) {
//     ros::init(argc, argv, "object_detection_node");
//     ros::NodeHandle nh_;
//     ROSNode rosnode(nh_);
//     ros::spin();
//     return 0;
// }
