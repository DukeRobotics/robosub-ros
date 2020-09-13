#!/bin/bash

set -ex

docker-compose run --entrypoint /bin/bash "${SERVICE_NAME}" -c "cd ~/dev/robosub-ros && ./build.sh ${SERVICE_NAME}"
