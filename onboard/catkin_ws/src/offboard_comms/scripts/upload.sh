#!/bin/bash

set -eu

ARD_TYPE=""
ARD_COMPILE_ONLY=false
SRC_CODE_PRESSURE=""
SRC_CODE_THRUSTER=""
PORT_PRESSURE=""
PORT_THRUSTER=""

# Function to print usage of this script
usage() {
    echo "Usage:"
    echo "    rosrun offboard_comms dual_upload.sh                               Only install Arduino core and ROS libraries."
    echo "    rosrun offboard_comms dual_upload.sh [pressure|thruster|dual]      Compile and upload code to the Arduino(s)."
    echo "    rosrun offboard_comms dual_upload.sh [pressure|thruster|dual] -c   Only compile the code, do not upload to the Arduino(s)."
    exit 0
}

# Make sure options are valid; print usage if not or if -h is passed
while getopts ":h" opt; do
    case ${opt} in
        h )
            usage
        ;;
        \? )
            echo "Invalid Option: -$OPTARG" 1>&2
            usage
        ;;
    esac
done

shift $((OPTIND - 1))

# Get the Arduino type from the first argument
if [ $# -gt 0 ]; then
    ARD_TYPE="$1"
    case $ARD_TYPE in
        pressure|thruster|dual)
            ;;
        *)
            echo "Invalid Arduino type: $ARD_TYPE. Valid values are pressure, thruster, or dual." 1>&2
            exit 1
            ;;
    esac
fi

# If -c is passed, only compile the code, do not upload to the Arduino(s)
if [ $# -ge 2 ] && [ -n "$2" ] && [ $2 = "-c" ]; then
    ARD_COMPILE_ONLY=true
else
    ARD_COMPILE_ONLY=false
fi

# Install ROS libraries for Arduino
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
zip -r ros_lib.zip ros_lib

export ARDUINO_LIBRARY_ENABLE_UNSAFE_INSTALL=true
arduino-cli lib install --zip-path ros_lib.zip
rm -f ros_lib.zip

# Install Arduino core libraries
arduino-cli core install arduino:megaavr
arduino-cli core install arduino:avr

PKG_DIR=$(rospack find offboard_comms)

# Compile Pressure Arduino code
if [ "$ARD_TYPE" = "pressure" ] || [ "$ARD_TYPE" = "dual" ]; then
    SRC_CODE_PRESSURE="${PKG_DIR}/Arduino Sketchbooks/PressureArduino"
    PORT_PRESSURE=$(rosrun offboard_comms port_finder.py pressure)
    arduino-cli compile -b arduino:avr:uno "$SRC_CODE_PRESSURE"
fi

# Compile Thruster Arduino code
if [ "$ARD_TYPE" = "thruster" ] || [ "$ARD_TYPE" = "dual" ]; then
    rosrun offboard_comms copy_offset.sh 1 # Copy the offset file to the compile folder
    SRC_CODE_THRUSTER="${PKG_DIR}/Arduino Sketchbooks/ThrusterArduino"
    PORT_THRUSTER=$(rosrun offboard_comms port_finder.py thruster)
    arduino-cli compile -b arduino:megaavr:nona4809 "$SRC_CODE_THRUSTER"
    rosrun offboard_comms copy_offset.sh 0 # Remove the offset file from the compile folder
fi

echo ""

# Print the port and source code path for Pressure Arduino
if [ "$ARD_TYPE" = "pressure" ] || [ "$ARD_TYPE" = "dual" ]; then
    echo "PORT_PRESSURE: $PORT_PRESSURE"
    echo "SRC_CODE_PRESSURE: $SRC_CODE_PRESSURE"
    echo ""
fi

# Print the port and source code path for Pressure Arduino
if [ "$ARD_TYPE" = "thruster" ] || [ "$ARD_TYPE" = "dual" ]; then
    echo "PORT_THRUSTER: $PORT_THRUSTER"
    echo "SRC_CODE_THRUSTER: $SRC_CODE_THRUSTER"
    echo ""
fi

# Upload the code to the Arduino(s) if -c is not passed
if [ "$ARD_COMPILE_ONLY" = false ]; then

    # Upload to Pressure Arduino
    if [ "$ARD_TYPE" = "pressure" ] || [ "$ARD_TYPE" = "dual" ]; then
        echo "Uploading to Pressure Arduino"
        arduino-cli upload -b arduino:avr:uno -p "$PORT_PRESSURE" "$SRC_CODE_PRESSURE"
        echo "Completed upload to Pressure Arduino."
        echo ""
    fi

    # Upload to Thruster Arduino
    if [ "$ARD_TYPE" = "thruster" ] || [ "$ARD_TYPE" = "dual" ]; then
        echo "Uploading to Thruster Arduino."
        arduino-cli upload -b arduino:megaavr:nona4809 -p "$PORT_THRUSTER" "$SRC_CODE_THRUSTER"
        echo "Completed upload to Thruster Arduino."
        echo ""
    fi

# If -c is passed, only compile the code, do not upload to the Arduino(s)
else
    if [ "$ARD_TYPE" = "pressure" ] || [ "$ARD_TYPE" = "dual" ]; then
        echo "Compiled Pressure Arduino code. Did not upload."
    fi

    if [ "$ARD_TYPE" = "thruster" ] || [ "$ARD_TYPE" = "dual" ]; then
         echo "Compiled Thruster Arduino code. Did not upload."
    fi

    echo ""
fi
