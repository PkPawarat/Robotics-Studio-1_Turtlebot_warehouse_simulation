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
    void setGoals(std::vector<geometry_msgs::Point> goals);

    /**
     * @brief Accepts the container of goals.
     *
     * @param goals
     */
    void GoToNext()

    /**
     * @brief The stored vector of goals for the mission class
     */
    std::vector<geometry_msgs::Point> Goals;

    /**
     * @brief The stored int to hold the step of the goal that mission is up to. This is set to zero at the class constructor
     */
    int GoalStep; 
 }
 