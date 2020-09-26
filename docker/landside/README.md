# Dockerfile Notes

This image is run on the landside computer during a pool test.

## Building

- Building should be done by the maintainer of the Dockerfile, and published so that others can follow the 'Running' instructions to use the image. This version is only designed to be built for amd64.

1. ```cd``` into the directory containing the Dockerfile

2. Build
```bash
docker build -t dukerobotics/robosub-ros:landside .
```


