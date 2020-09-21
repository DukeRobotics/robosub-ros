#!/bin/bash

set -ex

# Use experimental Docker
sudo jq -n '{experimental: true}' | sudo tee /etc/docker/daemon.json > /dev/null
sudo systemctl restart docker

docker run --rm --privileged multiarch/qemu-user-static --reset -p yes

# Build image
cd docker/"${SERVICE_NAME}"
docker build --build-arg BASE_IMAGE="${BASE_IMAGE}" --build-arg TARGETPLATFORM="${TARGETPLATFORM}" --platform "${TARGETPLATFORM}" -t "${IMAGE_NAME}" .
