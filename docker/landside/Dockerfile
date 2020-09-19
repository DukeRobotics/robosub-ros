ARG BASE_IMAGE=dukerobotics/robosub-ros:core
FROM ${BASE_IMAGE}

### SSH Settings ###

# Set ssh to not use local X11 so graphics are forwarded to user
RUN echo "X11UseLocalhost no" >> /etc/ssh/sshd_config && \
    # Set ssh to use port 2201
    echo "Port 2201" >> /etc/ssh/sshd_config

### Final Installs ###

# Update and install tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-melodic-geometry2 ros-melodic-rviz \
    ros-melodic-joy ros-melodic-rosbridge-suite \
    ros-melodic-image-view ros-melodic-image-publisher \
    xsltproc xauth && \
    apt-get clean && \
    # Clear apt caches to reduce image size
    rm -rf /var/lib/apt/lists/*

# Install the simulator
RUN wget coppeliarobotics.com/files/CoppeliaSim_Edu_V4_0_0_Ubuntu18_04.tar.xz && \
    tar -xf CoppeliaSim_Edu_V4_0_0_Ubuntu18_04.tar.xz && \
    rm CoppeliaSim_Edu_V4_0_0_Ubuntu18_04.tar.xz && \
    mv CoppeliaSim_Edu_V4_0_0_Ubuntu18_04 coppelia && \
    cp coppelia/compiledRosPlugins/libsimExtROSInterface.so coppelia/libsimExtROSInterface.so

### Final Setup ###

# Add script to setup multiple machines for pool testing
COPY setup_network.bash /opt/ros/melodic/setup_network.bash
RUN chmod +x /opt/ros/melodic/setup_network.bash

# Set computer type
ENV COMPUTER_TYPE=landside