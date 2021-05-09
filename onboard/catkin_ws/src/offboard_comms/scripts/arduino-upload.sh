#!/bin/bash

set -eux

rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
zip -r ros_lib.zip ros_lib

SRC_CODE="../Arduino Sketchbooks/ThrusterServoArduino"
PORT = $(./port_finder.sh)

arduino-cli lib install --zip-path ros_lib.zip
rm -f ros_lib.zip
arduino-cli core install arduino:avr
arduino-cli compile -b arduino:avr:nano ${SRC_CODE}
arduino-cli upload -b arduino:avr:nano -p ${PORT} ${SRC_CODE}
