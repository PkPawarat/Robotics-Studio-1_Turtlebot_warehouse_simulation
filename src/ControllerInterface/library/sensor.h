#pragma once
#include "ROSEnvironment.h"

// Keep only the headers needed
#include <vector>
// #include "pfms_types.h"
// #include "ros/ros.h"
#include <atomic>
#include <mutex>

#include "visualization_msgs/MarkerArray.h"
#include "std_srvs/SetBool.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseArray.h"


class Sensor {
private:
    // ROSEnvironment rosEnv;

public:
    Sensor();
    void simulateEnvironments();
    void detectObject();
    void detectQRCode();
};
