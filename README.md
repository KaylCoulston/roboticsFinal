Robotics Final Project
==========

Project Discription 
----

Our final robotics project will be split into three parts. These three parts consist of a robotic arm, navigation using SLAM, and vision processing. Our task for this final project will be to go to the FIZ, pick up a bin of tools and bring back that bin to Dana 3. We will be using SLAM to navigate to the FIZ, in the FIZ and back to Dana 3. Our robotic arm will be attached to the turtlebot. This arm will be capable of picking up a custom bin and holding until we reach our drop off location. Vision processing will be used to determine the correct bin that needs to be picked up.

 
The levels of deliverables are as follows:
 
  Level 1:
    Using SLAM to map IRLL and fiz
    Using SLAM to navigate to and from fiz
    Human puts tool in bin on top of rover
    Obstacle avoidance/dynamic path planning
 
  Level 2:
    Incorporate arm
    Vision processing for finding a bin
    Vision processing for picking up bin
 
  Level 3:
    Turtle bot knows how to self charge
    Face Recognition (human robot interaction with only certain humans aka fiz workers)
    Vision processing for locating specific bin
 
 ====================================

Instructions for running the code "go_to_specific_point_on_map.py", 

======================================

install the following Ros-library.

$ sudo apt-get install ros-indigo-move-base-msgs

In order to use it:

Connect the turtlebot to the workstation

Then ssh into the turtlebot and type the following command:

$ roslaunch turtlebot_bringup minimal.launch

in another terminal (ssh into the turtlebot)

$ roslaunch turtlebot_navigation amcl_demo.launch map_file:=<location of the map in the turtlebot>

on the workstation open a terminal and type the following:

$ roslaunch turtlebot_rviz_launchers view_navigation.launch --screen

Select "2D pose estimate" and select the direction of the of the turtlebot

Then we are ready to go :

edit the code by giving in the set co ordinates by using "publish point" from rviz

then in a new terminal:

$ python /go_to_specific_point_on_map.py

========================================

 

