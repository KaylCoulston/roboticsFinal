RoboAssistant
==========

Project Description 
----

Our robotics final project will be split into three parts. These three parts consist of a robotic arm, navigation using SLAM, and vision processing. Our task for this final project will be to go to the FIZ, pick up a bin of tools and bring back that bin to Dana 3. We will be using SLAM to navigate to the FIZ, in the FIZ and back to Dana 3. Our robotic arm will be attached to the turtlebot. This arm will be capable of picking up a custom bin and holding until we reach our drop off location. Vision processing will be used to determine the correct bin that needs to be picked up.

 
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
 


Instructions for sending robot to a point in the map
---------------

Install the following dependencies

`$ sudo apt-get install ros-indigo-move-base-msgs`

Instructions to run test it:

A map is required to be already saved on the turtlebot station
Connect the turtlebot to the workstation
Then `ssh` into the turtlebot and type the following command:

`$ roslaunch turtlebot_bringup minimal.launch`

in another terminal (`ssh` into the turtlebot)

`$ roslaunch turtlebot_navigation amcl_demo.launch map_file:=<location of the map in the turtlebot.yaml>`

on the workstation open a terminal and type the following:

`$ roslaunch turtlebot_rviz_launchers view_navigation.launch --screen`

Select "2D pose estimate" and select the direction of the of the turtlebot

Then we are ready to go :

edit the code by giving in the set coordinates by using "publish point" from `rviz`

then in a new terminal:

`$ python /go_to_specific_point_on_map.py`



Instructions for using AR Tags
---------------

NOTE: We assume you have already installed and setup the Turtlebot. Instructions taken from this [blog](http://ros-robotics.blogspot.com/2015/04/recognize-ar-tags-with-ros.html).

Install `AR_Track_Alvar` in the Turtlebot Laptop

`$ sudo apt-get install ros-indigo-ar-track-alvar`

Create a launch file

```
$ roscd turtlebot_bringup
$ cd launch
$ sudo nano alvar.launch
```

Copy and paste the following code to `alvar.launch` file

```
<launch>
    <!--If you are not using gmapping, UNCOMMENT the following line.
        However, if you are using gmapping, you need to run amcl_demo.launch before
        running alvar.launch-->
    <!--<include file="$(find turtlebot_bringup)/launch/3dsensor.launch"/>-->

    <arg name="marker_size"          default="5.0" />
    <arg name="max_new_marker_error" default="0.05" />
    <arg name="max_track_error"      default="0.05" />
    <arg name="cam_image_topic"      default="/camera/depth_registered/points" />
    <arg name="cam_info_topic"       default="/camera/rgb/camera_info" />
    <arg name="output_frame"         default="/camera_rgb_optical_frame" />

    <node name="ar_track_alvar" pkg="ar_track_alvar" type="individualMarkers" respawn="false" output="screen" args="$(arg marker_size) $(arg max_new_marker_error) $(arg max_track_error) $(arg cam_image_topic) $(arg cam_info_topic) $(arg output_frame)" />
</launch>
```

You can then use this launch file to detect AR tags by using the command

`$ roslaunch turtlebot_bringup alvar.launch`

To check the output, use the following command. Place an AR tag in front of the kinect camera and it should detect the `Tag ID` and it's `x, y, z`

`$ rostopic echo /ar_pose_marker`

To create AR Tags, use the following command. It'll prompt for the AR Tag ID you want to create and it will create a `.png` file on the current directory you are running the command. Just print it!

`$ rosrun ar_track_alvar createMarker`



Instructions for auto-docking
---------------

Docking station must be 3 meters in line of sight for the Turtlebot to successfully dock. We assume that `minimal.launch` is already running. Instructions from [here](http://learn.turtlebot.com/2015/02/01/12/).

On the workstation run:

`$ roslaunch kobuki_auto_docking minimal.launch --screen`

On a new terminal run:

`$ roslaunch kobuki_auto_docking activate.launch --screen`

