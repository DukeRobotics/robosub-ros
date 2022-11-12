#!/bin/bash

# A script that automatically formats code to the standard.

# Run python formatter
# shellcheck disable=SC2035
autopep8 -r -i --aggressive --aggressive --max-line-length 120 \
    --exclude **/sim.py,**/simConst.py \
    core/ros2_ws/src \
    onboard/ros2_ws/src \
    landside/ros2_ws/src \
    scripts
