 //Mission class that can assign a pickup object and dropoff location

#include <cmath>
#include <chrono>
#include <tuple>
#include <mission.h>

//Empty constructor
Mission::Mission(){
GoalStep = 0;
}

~Mission(){};


 void setGoals(std::vector<geometry_msgs::Point> goals){
    Goals = goals;
 }

void GoToNext(){
    
}