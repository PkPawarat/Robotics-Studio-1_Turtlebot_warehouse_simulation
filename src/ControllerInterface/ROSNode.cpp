
#include "library/ROSNode.h"
// Keep only the headers needed
#include "ros/ros.h"


ROSNode::ROSNode(ros::NodeHandle nh) : nh_(nh){
    odom = nh_.subscribe("/odom", 1, &ROSNode::odomCallBack, this);
    laser_scan = nh_.subscribe("/scan", 1, &ROSNode::laserScanCallBack, this);
    camera = nh_.subscribe("/camera/rgb/image_raw", 1, &ROSNode::cameraCallBack, this);
    camera_depth = nh_.subscribe("/camera/depth/points", 1, &ROSNode::pointCloudCallBack, this);
    pub_vel = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 3, false);
    pub_goal = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 3, false);
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
    ROS_INFO_STREAM(point_cloud);
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
    point_cloud = *msg;


    // pcl::PCLPointCloud2 pcl_cloud;
    // pcl_conversions::toPCL(point_cloud, pcl_cloud);

    // pcl::PointCloud<pcl::PointXYZ> pcl_xyz_cloud;
    // pcl::fromPCLPointCloud2(pcl_cloud, pcl_xyz_cloud);

    // std::vector<Point> temp_;
    // // Now, you can access the points in pcl_xyz_cloud
    // for (const pcl::PointXYZ& point : pcl_xyz_cloud.points)
    // {
    //     Point point_coords;
        
    //     point_coords.x = point.x;
    //     point_coords.y = point.y;
    //     point_coords.z = point.z;

    //     temp_.push_back(point_coords);

    // }

    // pcl_points = temp_;
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


void ROSNode::sendGoal(geometry_msgs::Pose position)
{
    std::string map = "map";
    bot_goal.header.frame_id = map;  // this need to change according to the rostopic echo /move_base_simple/goal 
    bot_goal.pose = position;
    pub_goal.publish(bot_goal);
}