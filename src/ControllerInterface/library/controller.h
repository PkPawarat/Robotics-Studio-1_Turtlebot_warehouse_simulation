#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "sensor.h"
#include <string>
#include "controllerinterface.h"
#include "pathfinding.h"
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
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"

#include "ROSNode.h"


/*!
 *  \brief     Controller Class
 *  \details
 *  Brief Description: The "Controller" class serves as a foundational template for developing autonomous vehicle controller classes. 
 *  It is intended for use as a base class, and its methods, as defined in the interface class, are crucial for implementing and testing autonomous vehicle control.
 *  \author    Pawarat Phatthanaphusakun
 *  \version   1.01
 *  \date      2023-08-28
 *  \pre       none
 *  \bug
 *  \warning   EVERYONE MUST NOT change this class (the header file)
 */

class Controller : public ControllerInterface {
    private:
        // Sensor sensor;
        bool targetDetected;
        bool qrCodeDetected;
        bool obstacleDetected;
        bool batteryLevel;
        bool LIDARreading;
        int battery;
        std::string currentTarget;

        PathPlanning _pathPlanning;

        std::vector<Node> _node;

    public:
        Controller(ROSNode*  rn);
        virtual void SetTargets(std::vector<geometry_msgs::Point>);
        virtual void Execute();
        virtual void CheckTarget();
        virtual void AssignTarget(const std::string& target);
        virtual void CheckQRCode();
        
        virtual void DriveTo(geometry_msgs::Point target);
        virtual void TurnTo(geometry_msgs::Point target);

        virtual void PickUpTarget();
        virtual void DropTarget();

        virtual void CheckObstacle();
        virtual void CheckBattery();
        virtual void Stop();
        virtual void Charge();


        virtual void RePerentObject();


        virtual double GetRotationTo(geometry_msgs::Point target);
        
        int CountTargets(); 
        
        void SetPathPlanning(PathPlanning pathPlanning, std::vector<Node> node);
        
        std::mutex mtx;
        void ThreadedExecute();

        void StartExecute();



        
    protected:
        std::vector<geometry_msgs::Point> Targets;

        geometry_msgs::PoseStamped Goal;
        ROSNode* ROSNode_; 
    };

#endif // CONTROLLER
