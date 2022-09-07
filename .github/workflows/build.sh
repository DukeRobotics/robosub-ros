#!/bin/bash

set -ex

# Use experimental Docker
sudo apt-get update
sudo systemctl restart docker

# Build image
cd docker/"${SERVICE_NAME}"
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
docker buildx create --name multiarch --driver docker-container --use --bootstrap
docker buildx inspect --bootstrap
docker buildx build --build-arg BASE_IMAGE="${BASE_IMAGE}" --build-arg TARGETPLATFORM="${TARGETPLATFORM}" --build-arg CUDA="${CUDA}" --platform="${TARGETPLATFORM}" --load -t"${IMAGE_NAME}" .
docker images
