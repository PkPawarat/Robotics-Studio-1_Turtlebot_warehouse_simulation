#ifndef CONTROLLERINTERFACE_H
#define CONTROLLERINTERFACE_H

#include <vector>
// #include "pfms_types.h"

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

    // /**
    // Run controller in reaching goals - non blocking call
    // */
    // virtual void run(void) = 0;

    // /**
    // Setter for goals
    // @param goals      #TODO: need to use grid instead of point location
    // @return all goal reachable, in order supplied
    // */
    // virtual bool setGoals(geometry_msgs::PoseArray goals) = 0;

    // /**
    // Checks whether the platform can travel between origin and destination
    // #TODO: need to use grid to find a paths location.
    // */
    // virtual bool checkOriginToDestination(nav_msgs::Odometry origin,
    //                                       geometry_msgs::Point goal,
    //                                       double &distance,
    //                                       double &time,
    //                                       nav_msgs::Odometry &estimatedGoalPose) = 0;

    // /**
    // Getter for distance to be travelled to reach current goal
    // @return distance to be travlled to reach current goal [m]
    // */
    // virtual double distanceToGoal(void) = 0;

    // /**
    // Getter for time to reach current goal
    // @return time to travel to current goal [s]
    // */
    // virtual double timeToGoal(void) = 0;

    // /**
    // Set tolerance when reaching goal
    // @return tolerance accepted [m]
    // */
    // virtual bool setTolerance(double tolerance) = 0;

    // /**
    // returns distance travelled by platform
    // @return total distance travelled since execution @sa run called with goals supplied
    // */
    // virtual double distanceTravelled(void) = 0;

    // /**
    // returns total time in motion by platform
    // @return total time in motion since execution @sa run called with goals supplied
    // */
    // virtual double timeTravelled(void) = 0;

    // /**
    // returns current odometry information
    // @return odometry - current odometry
    // */
    // virtual nav_msgs::Odometry getOdometry(void) = 0;

    virtual void Execute() = 0 ;
    virtual void CheckTarget() = 0 ;
    virtual void AssignTarget(const std::string &target) = 0 ;
    virtual void CheckQRCode() = 0 ;
    virtual void DriveTo(const std::string &location) = 0 ;
    virtual void PickUpTarget() = 0 ;
    virtual void DropTarget() = 0 ;
};

#endif // CONTROLLERINTERFACE_H
