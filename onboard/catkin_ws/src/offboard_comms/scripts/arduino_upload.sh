#!/bin/bash

set -eux

rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
zip -r ros_lib.zip ros_lib

PKG_DIR=$(rospack find offboard_comms)
SRC_CODE="${PKG_DIR}/Arduino Sketchbooks/ThrusterServoArduino"
PORT=$(${PKG_DIR}/scripts/port_finder.sh)

export ARDUINO_LIBRARY_ENABLE_UNSAFE_INSTALL=true
arduino-cli lib install --zip-path ros_lib.zip
rm -f ros_lib.zip
arduino-cli core install arduino:avr
arduino-cli compile -b arduino:avr:nano:cpu=atmega328old "${SRC_CODE}"
arduino-cli upload -b arduino:avr:nano:cpu=atmega328old -p "${PORT}" "${SRC_CODE}"
