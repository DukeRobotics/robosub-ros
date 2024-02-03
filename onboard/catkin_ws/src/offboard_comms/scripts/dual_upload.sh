#!/bin/bash

set -eu

ARD_UPLOAD=true
while getopts ":hc" opt; do
    case ${opt} in
        h )
            echo "Usage:"
            echo "    rosrun offboard_comms dual_upload.sh                  Compile and upload code to the arduino"
            echo "    rosrun offboard_comms dual_upload.sh -c               Only compile arduino code (no upload)"
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

rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
zip -r ros_lib.zip ros_lib

PKG_DIR=$(rospack find offboard_comms)
SRC_CODE_PRESSURE="${PKG_DIR}/Arduino Sketchbooks/PressureArduino"
SRC_CODE_THRUSTER="${PKG_DIR}/Arduino Sketchbooks/ThrusterArduino"

PORT_PRESSURE=$(rosrun offboard_comms port_finder.py pressure)
PORT_THRUSTER=$(rosrun offboard_comms port_finder.py thruster)

export ARDUINO_LIBRARY_ENABLE_UNSAFE_INSTALL=true
arduino-cli lib install --zip-path ros_lib.zip
rm -f ros_lib.zip

rosrun offboard_comms copy_offset.sh 1 # Copy the offset file to the compile folder

arduino-cli core install arduino:megaavr
arduino-cli core install arduino:avr
arduino-cli compile -b arduino:avr:uno "$SRC_CODE_PRESSURE"
arduino-cli compile -b arduino:megaavr:nona4809 "$SRC_CODE_THRUSTER"

rosrun offboard_comms copy_offset.sh 0 # Remove the offset file from the compile folder

if [ "$ARD_UPLOAD" = true ]; then

    # Upload to Arduino boards
    echo ""
    echo "PORT_PRESSURE: $PORT_PRESSURE"
    echo "SRC_CODE_PRESSURE: $SRC_CODE_PRESSURE"
    echo ""
    echo "PORT_THRUSTER: $PORT_THRUSTER"
    echo "SRC_CODE_THRUSTER: $SRC_CODE_THRUSTER"
    echo ""

    echo "Uploading to Pressure Arduino"
    arduino-cli upload -b arduino:avr:uno -p "$PORT_PRESSURE" "$SRC_CODE_PRESSURE"
    echo "Upload to Pressure Arduino complete"
    echo ""
    echo "Uploading to Thruster Arduino"
    arduino-cli upload -b arduino:megaavr:nona4809 -p "$PORT_THRUSTER" "$SRC_CODE_THRUSTER"
    echo "Upload to Thruster Arduino complete"
    echo ""
fi
