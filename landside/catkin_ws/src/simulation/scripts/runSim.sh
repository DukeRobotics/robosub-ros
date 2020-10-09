#!/bin/bash

COPPELIA_SIM_SCRIPT=~/docker-build/coppelia/coppeliaSim.sh
SERVER="comm_server_scene.ttt"
echo "Starting CoppeliaSim"

SERVER_LOCATION=$(find /root/dev -name $SERVER | head -1)

if [ -z "$SERVER_LOCATION" ]
then
      echo "Server file not found"
else
      $COPPELIA_SIM_SCRIPT -h -s "$SERVER_LOCATION"
fi
