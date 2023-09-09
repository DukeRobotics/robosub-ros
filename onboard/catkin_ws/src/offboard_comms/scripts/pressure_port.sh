#!/bin/bash

PKG_DIR=$(rospack find offboard_comms)
"${PKG_DIR}"/scripts/devices.sh | grep 3FE1330851544B5933202020FF070938 -A2 | tail -n1 | tr -d '\n' | cat
