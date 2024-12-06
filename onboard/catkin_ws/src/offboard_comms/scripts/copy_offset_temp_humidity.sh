#!/bin/bash

# Copies the relevant offset .h file into the compile folder (tempHumidityArduino) before compiling and uploading to the Arduino.
# One argument: 1 to copy the offset file, 0 to remove the copied offset file.

ROBOT_NAME=$(printenv ROBOT_NAME)

if [ "$ROBOT_NAME" = "oogway" ] | [ "$ROBOT_NAME" = "crush" ] | [ "$ROBOT_NAME" = "oogway-shell" ]; then
    tempHumidity_file="$ROBOT_NAME" + "TempHumidity.h";
    offset_file="$ROBOT_NAME" + "oogwayThrusters.h"
else
    tempHumidity_file="oogwayTempHumidity.h"
    offset_file="oogwayThrusters.h"
    echo "WARN: ROBOT_NAME environment variable is not set to a valid value. Defaulting to oogwayTempHumidity.h and oogwayThruster.h"
fi

PKG_DIR=$(rospack find offboard_comms)
if [ "$2" = "copy" ]; then
    if [ "$1" = "peripheral" ]; then
        cp "${PKG_DIR}/Arduino Sketchbooks/${tempHumidity_file}" "${PKG_DIR}/Arduino Sketchbooks/PeripheralArduino/tempHumidity.h"
    elif [ "$1" = "thruster" ]; then
        cp "${PKG_DIR}/Arduino Sketchbooks/${offset_file}" "${PKG_DIR}/Arduino Sketchbooks/ThrusterArduino/offset.h"
        cp "${PKG_DIR}/Arduino Sketchbooks/${offset_file}" "${PKG_DIR}/include/offset.h"
    else
        echo "WARN: first argument (Arduino Type) must be set to either "peripheral" or "thruster""
    fi
elif [ "$2" = "remove" ]; then
    if [ "$1" = "peripheral" ]; then
        rm "${PKG_DIR}/Arduino Sketchbooks/PeripheralArduino/tempHumidity.h"
    elif [ "$1" = "thruster" ]; then
        rm "${PKG_DIR}/Arduino Sketchbooks/ThrusterArduino/offset.h"
        rm "${PKG_DIR}/include/offset.h"
    else
        echo "WARN: first argument (Arduino Type) must be set to either "peripheral" or "thruster""
    fi
else
    echo "WARN: second argument (Action) must be set to either "copy" or "remove""
fi
