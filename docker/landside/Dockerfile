ARG BASE_IMAGE=dukerobotics/robosub-ros:core
FROM ${BASE_IMAGE}

### SSH Settings ###

# Set ssh to use port 2201
RUN echo "Port 2201" >> /etc/ssh/sshd_config

### Final Installs ###

# Update and install tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-geometry2 ros-noetic-rviz \
    ros-noetic-joy ros-noetic-rosbridge-suite \
    ros-noetic-image-view ros-noetic-image-publisher \
    xsltproc xauth && \
    apt-get clean && \
    # Clear apt caches to reduce image size
    rm -rf /var/lib/apt/lists/*

### Final Setup ###

# Add script to setup multiple machines for pool testing
COPY setup_network.bash /opt/ros/noetic/setup_network.bash
RUN chmod +x /opt/ros/noetic/setup_network.bash

# Set computer type
ENV COMPUTER_TYPE=landside
RUN echo "COMPUTER_TYPE=landside" >> /root/.ssh/environment

# Set working directory to codebase
WORKDIR /root/dev/robosub-ros
