#ifndef CONTROLLER
#define CONTROLLER

#include "sensor.h"
#include <string>
#include "controllerinterface.h"


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

class Controller : public ControllerInterface {
    private:
        Sensor sensor;
        bool targetDetected;
        bool qrCodeDetected;
        bool obstacleDetected;
        bool batteryLevel;
        std::string currentTarget;

    public:
        Controller();
        virtual void Execute();
        virtual void CheckTarget();
        virtual void AssignTarget(const std::string& target);
        virtual void CheckQRCode();
        virtual void DriveTo(const std::string& location);
        virtual void PickUpTarget();
        virtual void DropTarget();
        virtual void CheckObstacle();
        virtual void CheckBattery();
        virtual void Stop();
        virtual void Charge();
    };

#endif // CONTROLLER
