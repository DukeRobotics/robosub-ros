#!/bin/bash

set -ex

sudo systemctl restart docker

# Build image
cd docker/"${SERVICE_NAME}"
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
docker buildx build --build-arg BASE_IMAGE="${BASE_IMAGE}" --build-arg TARGETPLATFORM="${TARGETPLATFORM}" --build-arg CUDA="${CUDA}" --platform="${TARGETPLATFORM}" --load -t"${IMAGE_NAME}" .
