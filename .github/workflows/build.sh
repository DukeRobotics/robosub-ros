#!/bin/bash

set -ex

sudo apt-get remove docker docker-engine docker.io containerd runc
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
sudo add-apt-repository \
   "deb [arch=amd64] https://download.docker.com/linux/ubuntu \
   $(lsb_release -cs) \
   stable"
sudo apt-get update
sudo apt-get install docker-ce=5:19.03.15~3-0~ubuntu-bionic docker-ce-cli=5:19.03.15~3-0~ubuntu-bionic containerd.io

# Use experimental Docker
sudo jq -n '{experimental: true}' | sudo tee /etc/docker/daemon.json > /dev/null
sudo systemctl restart docker

docker run --rm --privileged multiarch/qemu-user-static --reset -p yes

# Build image
cd docker/"${SERVICE_NAME}"
docker build --build-arg BASE_IMAGE="${BASE_IMAGE}" --build-arg TARGETPLATFORM="${TARGETPLATFORM}" --platform "${TARGETPLATFORM}" -t "${IMAGE_NAME}" .
