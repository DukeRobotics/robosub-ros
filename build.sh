#!/bin/bash
# A convenience script used to build our code within the workplace you are in .
# When used to clean, argument 1 should be clean, and argument 2, if present should be "all" specifying to clean all workspaces.
# If argument 2 is omitted when cleaning, then the current workspace will be cleaned.

set -e

# Handle cleaning
if [[ "$1" == "clean" ]]; then
  if [[ "$2" == "all" ]]; then
    cd onboard/ros2_ws
    rm -r build install log
    cd ../..
    cd landside/ros2_ws
    rm -r build install log
    cd ../..
    cd core/ros2_ws
    rm -r build install log
    cd ../..
    echo "clean all successful"
  fi
  if [[ -z "$2" ]]; then
    cd "${COMPUTER_TYPE}"/ros2_ws
    rm -r build install log
    cd ../..
    echo "clean ${COMPUTER_TYPE} successful"
  fi
  exit 0
fi

if [[ -z "$COMPUTER_TYPE" ]]; then
  echo "COMPUTER_TYPE variable not defined, please set to workspace and rerun this script."
  exit 1
fi

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

echo "Building core workspace"
cd core/ros2_ws
colcon build
# shellcheck disable=SC1091
source install/setup.bash
cd ../..

echo "Building ${COMPUTER_TYPE} workspace"
cd "${COMPUTER_TYPE}"/ros2_ws
colcon build --symlink-install
# shellcheck disable=SC1091
source install/setup.bash
cd ../..

echo "If you did not source this scipt, please run"
echo "source ${COMPUTER_TYPE}/ros2_ws/install/setup.bash"
