#!/bin/bash
# shellcheck disable=SC1090,SC1091

source /opt/ros/melodic/setup.bash

if [ -f /opt/ros/melodic/setup_network.bash ]
then
    source /opt/ros/melodic/setup_network.bash
fi

# We know this directory exists, since it was created in the dockerfile
# shellcheck disable=SC2164
cd /root/dev/robosub-ros

if [ -f "${COMPUTER_TYPE}"/catkin_ws/devel/setup.bash ]
then
    source "${COMPUTER_TYPE}"/catkin_ws/devel/setup.bash
fi
