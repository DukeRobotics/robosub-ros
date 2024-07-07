"""
This script performs underwater color correction on images from a DepthAI camera.
It must be run on the DepthAI device itself, using a script node in the pipeline.
See depthai_spatial_detection.py for an example.
"""

# pyright: reportMissingImports=false, reportUnusedImport=false, reportUnboundVariable=false, reportUndefinedVariable=false
# flake8: noqa

while True:
    node.warn('Performing NN inferencing')
    frame = node.io['cam_rgb'].get()
    node.io['spatial_detection_network'].send(frame)
