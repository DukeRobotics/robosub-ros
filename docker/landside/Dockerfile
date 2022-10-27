ARG BASE_IMAGE=dukerobotics/robosub-ros:core
FROM ${BASE_IMAGE}

### SSH Settings ###

# Set ssh to use port 2201
RUN echo "Port 2201" >> /etc/ssh/sshd_config

### Final Installs ###

# Update and install tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-joy \
    xsltproc xauth && \
    apt-get clean && \
    # Clear apt caches to reduce image size
    rm -rf /var/lib/apt/lists/*

### Final Setup ###

# Set computer type
ENV COMPUTER_TYPE=landside
RUN echo "COMPUTER_TYPE=landside" >> /root/.ssh/environment

# Set working directory to codebase
WORKDIR /root/dev/robosub-ros
