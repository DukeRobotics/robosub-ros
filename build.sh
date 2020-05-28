#!/bin/bash
# A convenience script used to build our code. Takes one argument that specifies the workspace to build.

set -e

source /opt/ros/melodic/setup.bash

cd core/catkin_ws
catkin build
source devel/setup.bash
cd ../..

cd "$1"/catkin_ws
catkin build
source devel/setup.bash
