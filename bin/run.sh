#!/bin/bash
cd "$(dirname "$0")"
source /opt/ros/indigo/setup.bash
source devel/setup.bash
roslaunch hector_quadrotor_demo simple_gazebo.launch
