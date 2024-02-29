#!/bin/bash

PKG_DIR=$(rospack find offboard_comms)
"${PKG_DIR}"/scripts/devices.sh | grep 2E6BEE3251514A4E43202020FF131D41 -A2 | tail -n1 | tr -d '\n' | cat
