#!/bin/bash

set -e

source onboard/catkin_ws/devel/setup.bash
rosrun offboard_comms arduino_upload.sh -c
