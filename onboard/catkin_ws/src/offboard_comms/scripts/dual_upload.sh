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

onboard/catkin_ws/src/offboard_comms/scripts/copy_offset.sh 1 # Copy the offset file to the compile folder

rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
zip -r ros_lib.zip ros_lib

PKG_DIR=$(rospack find offboard_comms)
SRC_CODE1="${PKG_DIR}/Arduino Sketchbooks/PressureArduino"
SRC_CODE2="${PKG_DIR}/Arduino Sketchbooks/ThrusterArduino"
IFS=$'\n' read -r -d '' -a ARD_DEVS < <("${PKG_DIR}"/scripts/devices.sh && printf '\0')
ARDUINO1="${ARD_DEVS[0]}"
PORT1="${ARD_DEVS[2]}"
PORT2="${ARD_DEVS[3]}"

export ARDUINO_LIBRARY_ENABLE_UNSAFE_INSTALL=true
arduino-cli lib install --zip-path ros_lib.zip
rm -f ros_lib.zip

arduino-cli core install arduino:megaavr
arduino-cli compile -b arduino:megaavr:nona4809 "$SRC_CODE1"
arduino-cli compile -b arduino:megaavr:nona4809 "$SRC_CODE2"
if [ "$ARD_UPLOAD" = true ]; then
    # E49AFA8B51514C4B39202020FF024242 is thruster arduino
    # FAD98F6E51514C4B39202020FF020B42 is pressure arduino
    if [ "$ARDUINO1" = "E49AFA8B51514C4B39202020FF024242" ]; then
        TEMP=$SRC_CODE1
        SRC_CODE1=$SRC_CODE2
        SRC_CODE2=$TEMP
    fi

    # Upload to Arduino boards
    echo "PORT1: $PORT1"
    echo "PORT2: $PORT2"
    echo "SRC_CODE1: $SRC_CODE1"
    echo "SRC_CODE2: $SRC_CODE2"
    arduino-cli upload -b arduino:megaavr:nona4809 -p "$PORT1" "$SRC_CODE1"
    arduino-cli upload -b arduino:megaavr:nona4809 -p "$PORT2" "$SRC_CODE2"
fi

# onboard/catkin_ws/src/offboard_comms/scripts/copy_offset.sh 0 # Remove the offset file from the compile folder