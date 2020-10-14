#!/bin/bash
# A convenience script used to build our code. Takes one argument that specifies the workspace to build.

set -e

# Print help message on invalid argument
if [[ "$1" != "onboard" ]] && [[ "$1" != "landside" ]]; then
    echo "ERROR. Invalid argument or no argument specified. Please specify either onboard or landside, as in:"
    echo "./build.sh onboard"
    echo "./build.sh landside"
    exit 1
fi

# shellcheck disable=SC1091
source /opt/ros/melodic/setup.bash

cd core/catkin_ws
catkin build
# shellcheck disable=SC1091
source devel/setup.bash
cd ../..

cd "$1"/catkin_ws
catkin build
# shellcheck disable=SC1091
source devel/setup.bash
cd ../..

echo "If you did not source this scipt, please run"
echo "source ${1}/catkin_ws/devel/setup.bash"
