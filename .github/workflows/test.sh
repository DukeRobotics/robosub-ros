#!/bin/bash

set -ex

docker-compose run --entrypoint /bin/bash "${SERVICE_NAME}" -c cd ~/dev && \
                                ls -R && \
                                git clone https://github.com/DukeRobotics/robosub-ros.git && \
                                cd robosub-ros && \
                                git checkout ${github.sha} && \
                                ./build.sh ${SERVICE_NAME}
