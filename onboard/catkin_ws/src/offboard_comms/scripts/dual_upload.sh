#!/bin/bash

set -eu

# Check if arduino-cli is installed
if ! command -v arduino-cli &>/dev/null; then
    echo "Error: arduino-cli not found. Please install arduino-cli before running this script."
    exit 1
fi

ARD_UPLOAD=true
while getopts ":hc" opt; do
    case ${opt} in
        h )
            echo "Usage:"
            echo "    $0                  Compile and upload code to the arduino"
            echo "    $0 -c               Only compile arduino code (no upload)"
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

# Remove ros_lib directory and zip file if they exist
if [ -d "ros_lib" ]; then
    rm -rf ros_lib
fi
if [ -f "ros_lib.zip" ]; then
    rm -f ros_lib.zip
fi

# Generate ros_lib
rosrun rosserial_arduino make_libraries.py .
zip -r ros_lib.zip ros_lib

PKG_DIR=$(rospack find offboard_comms)
SRC_CODE1="${PKG_DIR}/Arduino Sketchbooks/PressureArduino"
SRC_CODE2="${PKG_DIR}/Arduino Sketchbooks/ThrusterArduino"
ARD_DEVS=$("${PKG_DIR}"/scripts/devices.sh)
ARDUINO1=$(echo "$ARD_DEVS" | awk 'NR==1{print}')
PORT1=$(echo "$ARD_DEVS" | awk 'NR==3{print}')
ARDUINO2=$(echo "$ARD_DEVS" | awk 'NR==2{print}')
PORT2=$(echo "$ARD_DEVS" | awk 'NR==4{print}')

export ARDUINO_LIBRARY_ENABLE_UNSAFE_INSTALL=true
arduino-cli lib install --zip-path ros_lib.zip
rm -f ros_lib.zip

arduino-cli core install arduino:megaavr

# Compile both Arduino sketches
arduino-cli compile -b arduino:megaavr:nona4809 "$SRC_CODE1"
arduino-cli compile -b arduino:megaavr:nona4809 "$SRC_CODE2"

if [ "$ARD_UPLOAD" = true ]; then
    # Swap Arduino and Port if needed
    if [ "$ARDUINO1" = "E49AFA8B51514C4B39202020FF024242" ]; then
        TEMP=$ARDUINO1
        ARDUINO1=$ARDUINO2
        ARDUINO2=$TEMP
        
        TEMP=$PORT1
        PORT1=$PORT2
        PORT2=$TEMP
    fi

    # Upload to Arduino boards
    arduino-cli upload -b arduino:megaavr:nona4809 -p "$PORT1" "$SRC_CODE1"
    arduino-cli upload -b arduino:megaavr:nona4809 -p "$PORT2" "$SRC_CODE2"
fi

