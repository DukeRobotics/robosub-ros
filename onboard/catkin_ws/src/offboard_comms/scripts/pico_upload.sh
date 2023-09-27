#!/bin/bash

# Make a new directory for the build
#

set -eu

ARD_UPLOAD=true
while getopts ":hc" opt; do
    case ${opt} in 
        h )
            echo "Usage:"
            echo "    rosrun offboard_comms upload_arduino.sh                  Compile and upload code to the arduino"
            echo "    rosrun offboard_comms upload_arduino.sh -c               Only compile arduino code (no upload)"
            exit 0
        ;;
        c )
            ARD_UPLOAD=false
        ;;
        \? )
            echo "Invalid Option -$OPTARG" 1>&2
            exit 1
        ;;
    esac
done

if [ "$ARD_UPLOAD" = true ]; then
    echo "Options parsed: compiling and uploading to the Arduino"
else
    echo "Options parsed: only compiling"
fi

# PKG_DIR="/root/dev/robosub-ros/onboard/catkin_ws/src/offboard_comms"
PKG_DIR="/home/robot/robosub-ros/onboard/catkin_ws/src/offboard_comms" #outside Docker
SRC_CODE="${PKG_DIR}/pico/pico.ino"
PORT=$("${PKG_DIR}"/scripts/port_finder.sh)

echo $PORT

export ARDUINO_LIBRARY_ENABLE_UNSAFE_INSTALL=true

arduino-cli lib install TinyGPSPlus

arduino-cli core install arduino:mbed_rp2040
arduino-cli compile -b arduino:mbed_rp2040:pico "${SRC_CODE}"

if [ "$ARD_UPLOAD" = true ]; then
    echo $PORT
    arduino-cli upload -p "${PORT}" -b arduino:mbed_rp2040:pico "${SRC_CODE}"
fi

echo "Pico upload complete."
