#!/bin/bash

set -ex

docker buildx create --name robobuilder --driver docker-container --use
docker buildx inspect --bootstrap
docker buildx ls
