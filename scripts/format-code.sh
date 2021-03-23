#!/bin/bash

# A script that automatically formats code to the standard.

# Run python formatter
# shellcheck disable=SC2035
autopep8 -r -i --aggressive --aggressive --max-line-length 120 \
    --exclude **/sim.py,**/simConst.py \
    core/catkin_ws/src \
    onboard/catkin_ws/src \
    landside/catkin_ws/src
