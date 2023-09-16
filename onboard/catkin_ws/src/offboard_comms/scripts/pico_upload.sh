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

rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
zip -r ros_lib.zip ros_lib

PKG_DIR=$(rospack find offboard_comms)
SRC_CODE="${PKG_DIR}/pico/pico.cpp"
PORT=$("${PKG_DIR}"/scripts/port_finder.sh)

export ARDUINO_LIBRARY_ENABLE_UNSAFE_INSTALL=true
arduino-cli lib install --zip-path ros_lib.zip
rm -f ros_lib.zip
arduino-cli core update-index --aditional-urls https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
arduino-cli compile -b --fqbn rp2040:rp2040:rpipico "${SRC_CODE}"

if [ "$ARD_UPLOAD" = true ]; then
    arduino-cli upload -p "${PORT}" --fqbn rp2040:rp2040:rpipico kr1000 "${SRC_CODE}"
fi