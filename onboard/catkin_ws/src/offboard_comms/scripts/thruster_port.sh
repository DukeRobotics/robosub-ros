#!/bin/bash

PKG_DIR=$(rospack find offboard_comms)
"${PKG_DIR}"/scripts/devices.sh | grep 393DA45D51514A4E43202020FF13152A -A2 | tail -n1 | tr -d '\n' | cat
