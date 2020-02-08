#!/bin/bash

CATKINDIR=~/dev/robosub-ros/catkin_ws
source /opt/ros/kinetic/setup.bash
cd $CATKINDIR
catkin build
echo "" && echo "Sourcing $CATKINDIR/devel/setup.bash..."
source $CATKINDIR/devel/setup.bash
echo "" && echo "Done building!" && echo ""
