#!/bin/bash
cd "$(dirname "$0")"
source /opt/ros/indigo/setup.bash
source devel/setup.bash
roslaunch simple_launch simple_gazebo.launch
