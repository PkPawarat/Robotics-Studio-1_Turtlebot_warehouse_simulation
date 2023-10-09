#ifndef CONTROLLERINTERFACE_H
#define CONTROLLERINTERFACE_H

#include <vector>
#include "ros/ros.h"
#include <atomic>
#include <mutex>

// Keep only the headers needed
#include "visualization_msgs/MarkerArray.h"
#include "std_srvs/SetBool.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseArray.h"

/*!
 *  \brief     Controller Interface Class
 *  \details
 *  This interface class is used to set all the methods that need to be embodies within any subsequent derived autonomous vehicle controller classes.
 *  The methods noted in interface class are the only methods that will be visible and used for testing the implementation of your code.
 *  \author    Pawarat Phatthanaphusakun
 *  \version   1.01
 *  \date      2023-08-28
 *  \pre       none
 *  \bug
 *  \warning   EVERYONE MUST NOT change this class (the header file)
 */

class ControllerInterface
{
  public:
    ControllerInterface(){};
    virtual void SetTargets(std::vector<geometry_msgs::Point>) = 0 ;
    virtual void Execute() = 0 ;
    virtual void CheckTarget() = 0 ;
    virtual void AssignTarget(const std::string &target) = 0 ;
    virtual void CheckQRCode() = 0 ;
    virtual void DriveTo(const std::string &location) = 0 ;
    virtual void PickUpTarget() = 0 ;
    virtual void DropTarget() = 0 ;
    virtual void RePerentObject() = 0;
};

#endif // CONTROLLERINTERFACE_H
