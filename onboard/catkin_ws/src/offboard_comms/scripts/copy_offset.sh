#!/bin/bash

# Copies the relevant offset .h file into the compile folder (ThrusterArduino) before compiling and uploading to the Arduino.
# One argument: 1 to copy the offset file, 0 to remove the copied offset file.

ROBOT_NAME=$(printenv ROBOT_NAME)

if [ "$ROBOT_NAME" = "cthulhu" ]; then
    offset_file="cthulhuThrusterOffset.h"
    echo "Using cthulhuThrusterOffset.h"
elif [ "$ROBOT_NAME" = "oogway" ]; then
    offset_file="oogwayThrusterOffset.h"
    echo "Using oogwayThrusterOffset.h"
else
    offset_file="oogwayThrusterOffset.h"
    echo "WARN: ROBOT_NAME environment variable is not set to a valid value. Defaulting to oogwayThrusterOffset.h"
fi

PKG_DIR=$(rospack find offboard_comms)
if [ "$1" = 1 ]; then
    cp "${PKG_DIR}/Arduino Sketchbooks/${offset_file}" "${PKG_DIR}/Arduino Sketchbooks/ThrusterArduino/offset.h"
else
    rm "${PKG_DIR}/Arduino Sketchbooks/ThrusterArduino/offset.h"
fi
