#ifndef CONTROLLER
#define CONTROLLER

#include "sensor.h"
#include <string>
#include "controllerinterface.h"


// Keep only the headers needed
#include <vector>
#include "pfms_types.h"
#include "ros/ros.h"
#include <atomic>
#include <mutex>

#include "visualization_msgs/MarkerArray.h"
#include "std_srvs/SetBool.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseArray.h"

class Controller : ControllerInterface {
    private:
        Sensor sensor;
        bool targetDetected;
        bool qrCodeDetected;
        std::string currentTarget;

    public:
        Controller();
        void Execute();
        void CheckTarget();
        void AssignTarget(const std::string& target);
        void CheckQRCode();
        void DriveTo(const std::string& location);
        void PickUpTarget();
        void DropTarget();
    };

#endif // CONTROLLER
