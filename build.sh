#!/bin/bash
# A convenience script used to build our code within the workplace you are in .
# When used to clean, argument 1 should be clean, and argument 2, if present should be "all" specifying to clean all workspaces.
# If argument 2 is omitted when cleaning, then the current workspace will be cleaned.

set -e

# Handle cleaning
if [[ "$1" == "clean" ]]; then
  if [[ "$2" == "all" ]]; then
    cd onboard/catkin_ws
    catkin clean -y
    cd ../..
    cd landside/catkin_ws
    catkin clean -y
    cd ../..
    cd core/catkin_ws
    catkin clean -y
    cd ../..
    echo "clean all successful"
  fi
  if [[ -z "$2" ]]; then
    cd "${COMPUTER_TYPE}"/catkin_ws
    catkin clean -y
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
source /opt/ros/noetic/setup.bash

echo "Building core workspace"
cd core/catkin_ws
catkin build
# shellcheck disable=SC1091
source devel/setup.bash
cd ../..

echo "Building ${COMPUTER_TYPE} workspace"
cd "${COMPUTER_TYPE}"/catkin_ws
catkin build
# shellcheck disable=SC1091
source devel/setup.bash
cd ../..

# Build the acoustics spike detection library
cd /root/dev/robosub-ros/onboard/catkin_ws/src/acoustics/rust
. "$HOME/.cargo/env"
virtualenv .venv
. .venv/bin/activate
pip install maturin
maturin develop --debug
cd .venv/lib/python3.8/site-packages
mv ./spike_detection /usr/local/lib/python3.8/dist-packages/spike_detection
    
echo "If you did not source this script, please run"
echo "source ${COMPUTER_TYPE}/catkin_ws/devel/setup.bash"
