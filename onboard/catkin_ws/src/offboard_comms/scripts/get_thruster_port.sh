#!/bin/bash

# This is just used for debugging

if [ "$ROBOT_NAME" = "oogway" ]; then
    echo "Using oogway thruster port"

    port=$(cat /home/robot/robosub-ros/onboard/catkin_ws/src/offboard_comms/config/oogway.yaml | grep -A 1 thruster | grep ftdi | cut -d "'" -f 2 | cut -c 10-)

    serial_port=$(ls /dev/serial/by-id/ | grep $port)

    serial_port=$(readlink -f /dev/serial/by-id/$serial_port)

    echo "Serial port: $serial_port"

fi