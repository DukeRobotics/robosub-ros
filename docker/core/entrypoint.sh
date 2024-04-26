#!/bin/bash

# Add the robot name from the compose file to the bash_profile and bashrc
echo "export ROBOT_NAME=${ROBOT_NAME}" >> /root/.bash_profile
echo "export ROBOT_NAME=${ROBOT_NAME}" >> /root/.bashrc

# Start the ssh server
service ssh restart &

bash
