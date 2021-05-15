#!/bin/bash

COPPELIA_SIM_SCRIPT=/root/docker-build/coppelia/coppeliaSim.sh
SERVER="comm_server_scene.ttt"
echo "Starting CoppeliaSim"

SERVER_LOCATION=$(find "$(rospack find simulation)" -name $SERVER | head -1)

if [ -z "$SERVER_LOCATION" ]
then
      echo "Server file not found"
else
      xvfb-run --auto-servernum --server-num=1 $COPPELIA_SIM_SCRIPT -h -s "$SERVER_LOCATION"
fi
