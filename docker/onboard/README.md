# Dockerfile Notes

This image is run on board the robot itself.

## Tags
This Dockerfile is used to generate three differently tagged images.

1. `onboard-amd64`: A Docker image built to be run on development computers and the NUC (and any other 64 bit x86 system).
2. `onboard-arm64`: A Docker image built to be run on the Jetson.
3. `onboard`: A Docker manifest list. Pulling `onboard` will automatically choose between an ARM and x86 image depending on your system. This is the one you should use in most cases.