#!/bin/bash

source /opt/ros/kinetic/setup.bash
cd ~/dev/robosub-ros/catkin_ws
catkin build
source ~/dev/robosub-ros/catkin_ws/devel/setup.bash
echo "" && echo "Done building!" && echo ""
