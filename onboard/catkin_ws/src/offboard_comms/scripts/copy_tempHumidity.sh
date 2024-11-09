#!/bin/bash

# Copies the relevant offset .h file into the compile folder (tempHumidityArduino) before compiling and uploading to the Arduino.
# One argument: 1 to copy the offset file, 0 to remove the copied offset file.

ROBOT_NAME=$(printenv ROBOT_NAME)

if [ "$ROBOT_NAME" = "oogway" ]; then
    tempHumidity_file="oogwayTempHumidity.h";
elif [ "$ROBOT_NAME" = "crush" ]; then
    tempHumidity_file="crushTempHumidity.h";
else
    offset_file="oogwaytempHumidity.h"
    echo "WARN: ROBOT_NAME environment variable is not set to a valid value. Defaulting to oogwayTempHumidity.h"
fi

PKG_DIR=$(rospack find offboard_comms)
if [ "$1" = 1 ]; then
    cp "${PKG_DIR}/Arduino Sketchbooks/${tempHumidity_file}" "${PKG_DIR}/Arduino Sketchbooks/Arduino/tempHumidity.h"
else
    rm "${PKG_DIR}/Arduino Sketchbooks/Arduino/tempHumidity.h"
fi
