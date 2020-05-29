#!/bin/bash
# A convenience script used to build our code. Takes one argument that specifies the workspace to build.

# If sourcing file then return instead of exiting shell
if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
    trap "trap - ERR;return 1" ERR
else
    trap "trap - ERR;exit 1" ERR
fi

# Print help message on invalid argument
if [[ "$1" != "onboard" ]] && [[ "$1" != "landside" ]]; then
    echo "ERROR. Invalid argument or no argument specified. Please specify either onboard or landside, as in:"
    echo "source ./build.sh onboard"
    echo "source ./build.sh landside"
    false
fi

source /opt/ros/melodic/setup.bash

cd core/catkin_ws
catkin build
source devel/setup.bash
cd ../..

cd "$1"/catkin_ws
catkin build
source devel/setup.bash

# All commands successful, release trap on err
trap - ERR