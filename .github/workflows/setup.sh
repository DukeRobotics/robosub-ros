#!/bin/bash

set -ex

# Install buildx
mkdir -p ~/.docker/cli-plugins
wget -O ~/.docker/cli-plugins/docker-buildx https://github.com/docker/buildx/releases/download/v0.3.1/buildx-v0.3.1.linux-amd64
chmod a+x ~/.docker/cli-plugins/docker-buildx
docker buildx create --name robobuilder --driver docker-container --use
docker buildx inspect --bootstrap
docker buildx ls
