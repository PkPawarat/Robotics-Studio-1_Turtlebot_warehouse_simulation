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
            ▪ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
        

        ◦ check imu by. this is checking that turtlebot have launch a topic into note.
            rostopic echo /imu
