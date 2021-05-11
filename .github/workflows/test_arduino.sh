#!/bin/bash

set -e

# shellcheck disable=SC1091
source onboard/catkin_ws/devel/setup.bash
rosrun offboard_comms arduino_upload.sh -c
