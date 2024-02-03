#!/bin/bash

PKG_DIR=$(rospack find offboard_comms)
"${PKG_DIR}"/scripts/devices.sh | grep 557323233303516132A1 -A2 | tail -n1 | tr -d '\n' | cat
