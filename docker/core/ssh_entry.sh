#!/bin/bash
# shellcheck disable=SC1090,SC1091

source /opt/ros/noetic/setup.bash

if [ -f /opt/ros/noetic/setup_network.bash ]
then
    source /opt/ros/noetic/setup_network.bash
fi

# We know this directory exists, since it was created in the dockerfile
# shellcheck disable=SC2164
cd /root/dev/robosub-ros

if [ -f "${COMPUTER_TYPE}"/catkin_ws/devel/setup.bash ]
then
    source "${COMPUTER_TYPE}"/catkin_ws/devel/setup.bash
fi

# Alias to start foxglove websocket
alias fg-ws="roslaunch --screen foxglove_bridge foxglove_bridge.launch port:=8765"
alias thruster-test="rostopic pub -r 20 /controls/thruster_allocs custom_msgs/ThrusterAllocs '{allocs: [0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05]}'"

