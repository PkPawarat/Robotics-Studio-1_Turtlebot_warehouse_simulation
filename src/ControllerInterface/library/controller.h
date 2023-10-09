/**
 * @file controller.h
 * @author Pawarat Phatthanaphusakun, Maximilian Deda
 * @brief This header file covers the controller class and initialises all variables and functions used within it. It is used to control the robots motion and interactions
 * @version 0.1
 * @date 2023-10-08
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef CONTROLLER
#define CONTROLLER
#include "sensor.h"
#include <string>
#include "controllerinterface.h"
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

/**
 * @brief Initialises the Controller Class as a subsclass from the Parent ControllerInterface Class
 * 
 */

class Controller : public ControllerInterface {
    private:
        /**
         * @brief Initialises the sensor variable
         * 
         */
        Sensor sensor;

        /**
         * @brief Initialises the targetDetected variable as a boolean
         * 
         */
        bool targetDetected;

        /**
         * @brief Initialises the qrCodeDetected variable as a boolean
         * 
         */
        bool qrCodeDetected;

        /**
         * @brief Initialises the obstacleDetected variable as a boolean
         * 
         */
        bool obstacleDetected;

        /**
         * @brief Initialises the batteryLevel variable as a boolean
         * 
         */
        bool batteryLevel;

        /**
         * @brief Initialises the currentTarget variable as a string
         * 
         */
        std::string currentTarget;

    public:
        /**
         * @brief Construct a new Controller object
         * 
         */
        Controller();


        /**
         * @brief Initialises the SetTargets function
         * 
         */
        virtual void SetTargets(std::vector<geometry_msgs::Point>);

        /**
         * @brief Initialises the Execute function
         * 
         */
        virtual void Execute();

        /**
         * @brief Initialises the CheckTarget function
         * 
         */
        virtual void CheckTarget();

        /**
         * @brief Initialises the AssignTarget function
         * 
         * @param target Creates a constant string variable "target" for use in the function
         */
        virtual void AssignTarget(const std::string& target);

        /**
         * @brief Initialises the CheckQRCode function
         * 
         */
        virtual void CheckQRCode();

        /**
         * @brief Initialises the DriveTo function
         * 
         * @param location Creates a constant string variable "location" for use in the function
         */
        virtual void DriveTo(const std::string& location);

        /**
         * @brief Initialises the PickUpTarget function
         * 
         */
        virtual void PickUpTarget();

        /**
         * @brief Initialises the DropTarget function
         * 
         */
        virtual void DropTarget();

        /**
         * @brief Initialises the CheckObstacle function
         * 
         */
        virtual void CheckObstacle();

        /**
         * @brief Initialises the CheckBattery function
         * 
         */
        virtual void CheckBattery();

        /**
         * @brief Initialises the Stop function
         * 
         */
        virtual void Stop();

        /**
         * @brief Initialises the Charge function
         * 
         */
        virtual void Charge();

        /**
         * @brief Initialises the RePerentObject function
         * 
         */
        virtual void RePerentObject();

        
    protected:
        struct TargetStats {
            geometry_msgs::Point location; //! location of goal
            double distance; //! distance to goal
            double time; //! time to goal
        };
        std::vector<TargetStats> Targets;

        geometry_msgs::PoseStamped Goal;

    };

#endif // CONTROLLER
