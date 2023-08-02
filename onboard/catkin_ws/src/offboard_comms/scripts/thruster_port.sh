#!/bin/bash

PKG_DIR=$(rospack find offboard_comms)
"${PKG_DIR}"/scripts/devices.sh | grep E49AFA8B51514C4B39202020FF024242 -A2 | tail -n1 | tr -d '\n' | cat
