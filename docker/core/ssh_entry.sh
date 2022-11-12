#!/bin/bash
# shellcheck disable=SC1090,SC1091

source /opt/ros/humble/setup.bash

# We know this directory exists, since it was created in the Dockerfile
# shellcheck disable=SC2164
cd /root/dev/robosub-ros
