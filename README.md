# Robotics-Studio-1
41068-Robotics-Studio-1



Instruction how to use Ros, Gazebo, TurtleBot3

    • Run Gazebo environment.
        ◦ roslaunch gazebo_ros “Launch file name”
    • Run Turtlebot3 environment command:
        ◦ Launch Turtlebot3 with house.
            ▪ export TURTLEBOT3_MODEL=waffle_pi
            ▪ roslaunch turtlebot3_gazebo turtlebot3_house.launch 
        ◦ Launch teleoperate the TurtleBot3 with keyboard (Separate terminal). 
            ▪ export TURTLEBOT3_MODEL=waffle_pi
            ▪ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
        ◦ Launch rviz for Turtlebot3 in order to see the laser scan (Separate terminal). 
            ▪ roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
        ◦ Launch plot odometry using rqt_plot (Separate terminal).
            ▪ rqt_plot

                rqt_plot need to assign a topic of the simulation that is runing such as /odom/pose/pose/position
        ◦ TurtleBot3 also include a teleoperate
            c roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
        

        ◦ check imu by. this is checking that turtlebot have launch a topic into note.
            rostopic echo /imu



    - Mapping : Grid mapping

        ▪ export TURTLEBOT3_MODEL=waffle
        ▪ roslaunch turtlebot3_gazebo turtlebot3_world.launch
        ▪ roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
        To move around
        ▪ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

    - Mapping :RTAB-MAP

        ▪ export TURTLEBOT3_MODEL=waffle
        ▪ roslaunch turtlebot3_gazebo
        ▪ turtlebot3_world.launch

        ▪ export TURTLEBOT3_MODEL=waffle
        ▪ roslaunch rtabmap_demos
        ▪ demo_turtlebot3_navigation.launch


    Mapping saver
        sudo apt install ros-noetic-map-server
        rosrun map_server map_saver -f ~/map



        use map in gazebo

        sudo apt install ros-noetic-turtlebot3-navigation
        sudo apt install ros-noetic-navigation
        
        export TURTLEBOT3_MODEL=waffle
        roslaunch turtlebot3_gazebo turtlebot3_world.launch
        roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml

        in rviz if the location of the robot is differ from gazebo use 2D Pose Estimate in the Rviz on the top bar to indicate estimate location & rotation of the robot. 


## NOTICE: when pull this project from git delete Build folder then do this command in terminal (need to do everytime)
Create Cmake build

    mkdir build
    cd build/
    cmake ..
    make

