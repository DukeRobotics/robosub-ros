#!/bin/bash

CATKINDIR="$( cd "$(dirname "$0")" ; pwd -P )"/../..
source /opt/ros/kinetic/setup.bash
cd $CATKINDIR
catkin build
echo "" && echo "Sourcing $CATKINDIR/devel/setup.bash..."
source $CATKINDIR/devel/setup.bash
echo "" && echo "Done building!" && echo ""
