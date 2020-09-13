#!/bin/bash

set -ex

docker-compose --file docker/clone-and-build.test.yml run sut
