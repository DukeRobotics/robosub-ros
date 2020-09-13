#!/bin/bash

set -ex

docker-compose run --entrypoint /bin/bash "${SERVICE_NAME}" -c "cd ~/dev/robosub-ros && ls -R && ./build.sh ${SERVICE_NAME}"
