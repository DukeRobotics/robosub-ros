#!/bin/bash

# To be appended to /root/.bashrc
source /opt/ros/noetic/setup.bash

if [ -f /opt/ros/noetic/setup_network.bash ]
then
    source /opt/ros/noetic/setup_network.bash
fi

if [ -f "${COMPUTER_TYPE}"/catkin_ws/devel/setup.bash ]
then
    source "/root/dev/robosub-ros/${COMPUTER_TYPE}"/catkin_ws/devel/setup.bash
fi

# Alias to start foxglove websocket
alias fg-ws="roslaunch --screen foxglove_bridge foxglove_bridge.launch port:=8765"
alias thruster-test="rostopic pub -r 20 /controls/thruster_allocs custom_msgs/ThrusterAllocs '{allocs: [0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05]}'"

