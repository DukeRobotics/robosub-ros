#!/bin/bash

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

./copy_offset.sh 1 # Copy the offset file into the compile folder

rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
zip -r ros_lib.zip ros_lib

PKG_DIR=$(rospack find offboard_comms)
SRC_CODE="${PKG_DIR}/Arduino Sketchbooks/ThrusterArduino"
PORT=$("${PKG_DIR}"/scripts/port_finder.sh)

export ARDUINO_LIBRARY_ENABLE_UNSAFE_INSTALL=true
arduino-cli lib install --zip-path ros_lib.zip
rm -f ros_lib.zip
arduino-cli core install arduino:megaavr
arduino-cli compile -b arduino:megaavr:nona4809 "${SRC_CODE}"

if [ "$ARD_UPLOAD" = true ]; then
    arduino-cli upload -b arduino:megaavr:nona4809 -p "${PORT}" "${SRC_CODE}"
fi

./copy_offset.sh 0 # Remove the offset file from the compile folder
