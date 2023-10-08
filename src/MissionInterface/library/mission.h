#ifndef MISSION_H
#define MISSION_H

#include <chrono>
#include <thread>
#include <bits/stdc++.h>

/*!
 *  \brief     Mission Class
 *  \details
    Create a mission class that will be able to assign pickup object and drop off location.
 *  \author    Connor Keogh
 *  \version   1.00
 *  \date      2023-10-06
 *  \pre       none
 *  \bug       none reported as of 2023-05-02
 *  \warning   
 */
 class Mission
 {
    public:

    /**
    The Default Constructor taking no inputss
    */
    Mission();

    /**
    The Default deconstructor
    */
    ~Mission();


    /**
     * @brief Accepts the container of goals.
     *
     * @param goals
     */
    void setGoals(std::vector<pfms::geometry_msgs::Point> goals, pfms::PlatformType platform);


 }
 
 
 
 //Mission class that can assign a pickup object and dropoff location

#include <cmath>
#include <chrono>
#include <tuple>
#include <mission.h>

//Empty constructor
Mission::Mission(){}

~Mission(){};


 void setGoals(){

 }

 void 
 