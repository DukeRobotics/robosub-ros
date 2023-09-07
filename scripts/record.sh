#!/bin/bash

# Script to record a collection of helpful topics as a rosbag.
# Usage: ./record.sh <filename>

# We record the following 13 topics:
    # All 4 buoy detections:
        # Serpens Caput
        # Taurus
        # Auriga
        # Cetus
    # All 2 gate detections:
        # Abydos
        # Earth
    # Bounding box predictions
    # Sonar response
    # Sonar image
    # Sonar request
    # DepthAI camera feed
    # Mono camera feed
    # State

rosbag record /cv/front/buoy_abydos_serpenscaput /cv/front/buoy_abydos_taurus /cv/front/buoy_earth_auriga /cv/front/buoy_earth_cetus /cv/front/gate_abydos /cv/front/gate_earth /cv/front/detections/compressed /sonar/cv/response /sonar/image/compressed /sonar/request /camera/front/rgb/preview/compressed /camera/usb_camera/compressed state -O "$1"