# Robotics-Studio-1
41068-Robotics-Studio-1



Instruction how to use Ros, Gazebo, TurtleBot3

- Run Gazebo environment.
    
        roslaunch gazebo_ros “Launch file name”

- Run Turtlebot3 environment command:
    - Launch Turtlebot3 with house.
            
            export TURTLEBOT3_MODEL=waffle_pi
            roslaunch turtlebot3_gazebo turtlebot3_house.launch 

    - Launch teleoperate the TurtleBot3 with keyboard (Separate terminal). 
            
            export TURTLEBOT3_MODEL=waffle_pi
            roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

    - Launch rviz for Turtlebot3 in order to see the laser scan (Separate terminal). 

            roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch

    - Launch plot odometry using rqt_plot (Separate terminal).

            rqt_plot

            rqt_plot need to assign a topic of the simulation that is runing such as /odom/pose/pose/position

    ◦ TurtleBot3 also include a teleoperate

            roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
    

    ◦ check imu by. this is checking that turtlebot have launch a topic into note.

            rostopic echo /imu



- Mapping : Grid mapping

        ▪ export TURTLEBOT3_MODEL=waffle
        ▪ roslaunch turtlebot3_gazebo turtlebot3_world.launch
        ▪ roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping

- To move around

        ▪ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

        To capture map with save in  .pgm and .yaml which will be store in catkin folder.
        - rosrun map_server map_saver

    - Mapping :RTAB-MAP

        export TURTLEBOT3_MODEL=waffle
        roslaunch turtlebot3_gazebo
        turtlebot3_world.launch

        export TURTLEBOT3_MODEL=waffle
        roslaunch rtabmap_demos
        demo_turtlebot3_navigation.launch


- Mapping saver

        sudo apt install ros-noetic-map-server
        rosrun map_server map_saver -f ~/map



        use map in gazebo

        sudo apt install ros-noetic-turtlebot3-navigation
        sudo apt install ros-noetic-navigation
        
        export TURTLEBOT3_MODEL=waffle
        roslaunch turtlebot3_gazebo turtlebot3_world.launch
        roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml

in rviz if the location of the robot is differ from gazebo use 2D Pose Estimate in the Rviz on the top bar to indicate estimate location & rotation of the robot. 

To create a new branch in a Git repository, you can use the `git branch` command. Here are the steps to create a new branch:
1. Open a terminal window.
2. Navigate to the directory of your Git repository using the `cd` command. If you're not already in the repository's directory, you need to navigate to it first.

        cd /path/to/your/repository

3. Make sure you are on the branch from which you want to create a new branch. You can check your current branch using the `git branch` command. The branch with an asterisk (*) next to it is your current branch.

        git branch

4. To create a new branch, use the following command. Replace `new_branch_name` with the name you want to give to your new branch.

        git branch new_branch_name

- For example, if you want to create a branch named "feature-branch," you would run:

        git branch feature-branch

5. To switch to the newly created branch, you can use the `git checkout` command:

        git checkout new_branch_name

- Alternatively, you can use the shorthand command to create and switch to a new branch in one step:

        git checkout -b new_branch_name

- For example:

        git checkout -b feature-branch

Now, you have successfully created a new branch and switched to it. You can start working on your new branch, making changes, and committing them independently of the main branch or any other existing branch. Remember to commit your changes when you're ready, and you can push the new branch to a remote repository using `git push` if needed.



## Using our world 

        roslaunch turtlebot3_gazebo final_warehouse.launch


## Darknet_ros

Darknet_ros is an object detection tool used to identify select ojbects. This package will be used alongside our program to detect unknown obstacles and cease all operations.

To use this, clone the repository in your workspace alongside the Robotics Studio repository.

        git clone --recursive git@github.com:leggedrobotics/darknet_ros.git

To maximise performance, build in release mode:

        catkin_make -DCMAKE_BUILD_TYPE=Release

For users who have NVIDIA graphics cards, see github for more information: https://github.com/leggedrobotics/darknet_ros#installation 

To use the tool, run your gazebo model:

        roslaunch my_custom_world_gazebo large_warehouse.launch 

and then the package:

        roslaunch darknet_ros darknet_ros.launch
        

## NOTICE: when pull this project from git delete Build folder then do this command in terminal (need to do everytime)
Create Cmake build

    mkdir build
    cd build/
    cmake ..
    make

        learn from : https://emanual.robotis.com/docs/en/platform/turtlebot3/nav_simulation/


### Documentation

In source documentation is a must, we will examine all code submitted for supporting documentation. You also need the dox file (mainpage.dox) to indicate how the code will run/behave. You need to modiy the mainpage.dox included which does not have any specific documentation.

In ROS, doxygen documentation in generated using the `rosdoc_lite` tool. If you do not have the tool you can install it via `sudo apt-get install ros-noetic-rosdoc-lite` (replace noetic with melodic if on 18.04)

To generate the documentation'

```bash
cd ~/catkin_ws/src/Robotics-Studio-1
rosdoc_lite .
```

You will find the documentation inside doc folder.

```bash
firefox ~/catkin_ws/src/Robotics-Studio-1/doc/html/index.html 
```



Record ros bag 
        
        cd ~/catkin_ws/src/Robotics-Studio-1/logs               // record bag in this directory
        rosbag record -O pointLocation.bag /move_base_simple/goal
        rosbag info pointLocation.bag                          // check ros bag
        rosbag info pickupshelf.bag                          // check ros bag
