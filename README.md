# OBSTACLE AVOIDANCE project

This is the OBSTACLE AVOIDANCE project repository done for the Robot Programming course at Sapienza University of Rome. The goal of the project is to build a node
that computes a repulsive field from the local laser scans and modulates the /cmd_vel so that the robot does not clash with the wall if instructed to do so.

Here you can find the OBSTACLE AVOIDANCE package containing a CMakeLists.txt, a package.xml, and a /src folder in which the ROS node is located.
Note that to test this repository using your machine, you need to download this repo and put it into the /src folder of your Catkin workspace.

To create your Catkin workspace, please follow the next steps.

### CREATE YOUR CATKIN WORKSPACE
You can create a Catkin workspace by following these steps:

open TERMINAL

$ mkdir my_project_workspace

$ cd my_project_workspace/

$ mkdir src

$ cd src/

$ catkin_init_workspace

$ cd ..

$ catkin build

### ****************************** NOT NECESSARY FOR TESTING (START) ******************************

### HOW I CREATED MY OBSTACLE AVOIDANCE PACKAGE
Now, starting from the last executed command in your TERMINAL, you can create a Catkin package by following these steps:

$ cd src/

$ catkin_create_pkg my_obstacle_avoidance_package roscpp sensor_msgs std_msgs

$ cd ..

$ catkin build

$ source devel/setup.bash


### HOW I CREATED MY OBSTACLE AVOIDANCE NODE
Again, starting from the previously executed command in your TERMINAL, you can create a ros node by following these steps:

$ cd src/my_obstacle_avoidance_package/src/

$ emacs my_obstacle_avoidance_node.cpp


### ****************************** NOT NECESSARY FOR TESTING (END) ******************************


## TESTING

### PRELIMINARY ACTIONS
As I said before, you need to download this repo and put it into the /src folder of your Catkin workspace. The rename the package from "Obstacle-Avoidance-RP" to "my_obstacle_avoidance_package" in order to be coherent with the previous steps.
Then you have to build your workspace by following these steps:

open TERMINAL

$ cd my_project_workspace/

$ catkin build


### HOW TO TEST
Now your environment is set and you are ready to test the project by following these steps:

- Open a terminal and start the master:
  
   $ roscore
  
- Open another terminal and start the obstacle avoidance node:
  
   $ cd my_project_workspace/
   
   $ catkin build
  
   $ source devel/setup.bash
  
   $ rosrun my_obstacle_avoidance_package my_obstacle_avoidance_node
  
- Open another terminal and start the environment:
  
   $ cd
  
   $ cd Desktop/robotprogramming_2021_22-main/source/srrg2_workspace/src/srrg2_configs/navigation_2d/
  
   **Note that this is the path where you have saved the map(.world)**
  
   $ wsrp
  
   **Note that this is an alias in .bashrc for 'wsrp=source ~/Desktop/robotprogramming_2021_22-main/source/srrg2_workspace/devel/setup.bash'**
  
   $ rosrun stage_ros stageros cappero_laser_odom_diag_obstacle_2020-05-06-16-26-03.world

- Open another terminal and start the robot movement:
  
   $ rostopic pub /cmd_vel geometry_msgs/Twist -r 1 -- '[0.4, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
