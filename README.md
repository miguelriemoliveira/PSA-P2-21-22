# PSA-P2-21-22

Simulates a turtlebot with a depth camera and a velodyne in the scenario of the National Robotics Festival (FNR).

# How to use

First launch the gazebo FNR scenario:

    roslaunch psa_fnr gazebo.launch

To launch world wihtout obstacles use:

    roslaunch psa_fnr gazebo.launch world_name:=fnr_no_obstacles.world

Then run the script for spawning your robot:

    roslaunch psa_robot_description bringup.launch

And finally you can create your own python node to subscribe to images and point clouds and send velocity commands.

One example is 

    rosrun psa_robot_description driver_example.py

# Installation

sudo apt-get install ros-noetic-velodyne-description

sudo pip3 install rospkg