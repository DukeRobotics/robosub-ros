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
SRC_CODE1="${PKG_DIR}/Arduino Sketchbooks/PressureArduino"
SRC_CODE2="${PKG_DIR}/Arduino Sketchbooks/ThrusterArduino"
ARD_DEVS=$("${PKG_DIR}"/scripts/devices.sh)
ARDUINO1=$(echo $ARD_DEVS | awk '{print $1}')
PORT1=$(echo $ARD_DEVS | awk '{print $3}')
ARDUINO2=$(echo $ARD_DEVS | awk '{print $2}')
PORT2=$(echo $ARD_DEVS | awk '{print $4}')

export ARDUINO_LIBRARY_ENABLE_UNSAFE_INSTALL=true
arduino-cli lib install --zip-path ros_lib.zip
rm -f ros_lib.zip

arduino-cli core install arduino:megaavr
arduino-cli compile -b arduino:megaavr:nona4809 "$SRC_CODE1"
arduino-cli compile -b arduino:megaavr:nona4809 "$SRC_CODE2"
if [ "$ARD_UPLOAD" = true ]; then
	# 393DA45D51514A4E43202020FF13152A is thruster arduino
	# 3FE1330851544B5933202020FF070938 is pressure arduino
	if [ "$ARDUINO1" = "393DA45D51514A4E43202020FF13152A" ]; then
		TEMP=$PORT1
		SRC_CODE1=$PORT2
		SRC_CODE2=$TEMP
	fi
	arduino-cli upload -b arduino:megaavr:nona4809 -p "$PORT1" "$SRC_CODE1"
	arduino-cli upload -b arduino:megaavr:nona4809 -p "$PORT2" "$SRC_CODE2"
fi
