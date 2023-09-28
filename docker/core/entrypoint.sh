#!/bin/bash

# Add the robot name from the compose file to the bash profile
echo "export ROBOT_NAME=${ROBOT_NAME}" >> /root/.bash_profile

# Start the ssh server
service ssh restart &

bash
