#!/bin/bash

# Copies the relevant offset .h file into the compile folder (ThrusterArduino) before compiling and uploading to the Arduino.
# 1 argument: 1 if to copy the offset file, 0 to remove it

ROBOT_NAME=$(printenv ROBOT_NAME)

if [ "$ROBOT_NAME" = "cthulhu" ]; then
    offset_file="cthulhuThrusterOffset.h"
    echo "Using cthulhuThrusterOffset.h"
elif [ "$ROBOT_NAME" = "oogway" ]; then
    offset_file="oogwayThrusterOffset.h"
    echo "Using oogwayThrusterOffset.h"
else
    echo "Error: ROBOT_NAME environment variable is not set to a valid value"
    exit 1
fi

if [ "$1" = 1 ]; then
    cp onboard/catkin_ws/src/offboard_comms/Arduino\ Sketchbooks/"$offset_file" onboard/catkin_ws/src/offboard_comms/Arduino\ Sketchbooks/ThrusterArduino/offset.h
else
    rm onboard/catkin_ws/src/offboard_comms/Arduino\ Sketchbooks/ThrusterArduino/offset.h
fi
