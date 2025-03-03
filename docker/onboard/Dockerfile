ARG BASE_IMAGE=dukerobotics/robosub-ros:core

# Multistage build, allows us to store the built pytorch wheel in an image
# and retrieve it later, since compiling pytorch from source takes 6-12 hrs
FROM dukerobotics/jetson-pytorch as jetson-pytorch

FROM ${BASE_IMAGE}

ARG TARGETPLATFORM
ARG CUDA

### SSH Setup ###

# Set ssh to use port 2200
RUN echo "Port 2200" >> /etc/ssh/sshd_config

### CUDA setup for Jetson ###

COPY setup_jetson.sh setup_jetson.sh
RUN ./setup_jetson.sh

ENV TMP_PATH ${CUDA:+/usr/local/cuda/bin:}
ENV PATH ${TMP_PATH}${PATH}
ENV NVIDIA_VISIBLE_DEVICES ${CUDA:+all}
ENV NVIDIA_DRIVER_CAPABILITIES ${CUDA:+all}
ENV IGNITION_VERSION=citadel

### Basic Setup ###


# Add gazebo ignition repo
RUN apt-get update && \
    apt-get install -y --no-install-recommends wget lsb-release gnupg && \
    sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | sudo tee /etc/apt/28a0a75160794451c027b125a6edef1bf919d342

# Update and install tools
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    xauth \
    pciutils \
    libusb-dev \
    libqt5x11extras5 \
    unzip \
    zip \
    util-linux \
    ros-noetic-smach-ros \
    ros-noetic-rosserial \
    ros-noetic-rosserial-arduino \
    ros-noetic-vision-msgs \
    ffmpeg \
    nmap \
    iputils-ping \
    python3-pandas \
    ros-noetic-foxglove-bridge \
    nmap \
    ignition-fortress \
    libeigen3-dev \
    libudev-dev

# Get required python packages
RUN pip3 install --no-cache-dir \
    numpy==1.24.4 scipy pymba python-dateutil \
    pyserial dependency-injector \
    gputil saleae \
    bluerobotics-ping \
    roboflow \
    python-nmap \
    transforms3d \
    sympy \
    scikit-learn \
    opencv-python \
    jsonpickle \
    seaborn

# Install PyTorch and Torchvision
COPY torch_installer.sh torch_deps/torch_installer.sh
COPY --from=jetson-pytorch /root/torch-1.8.0a0+5c1479f-cp38-cp38-linux_aarch64.whl torch_deps/torch-1.8.0a0+5c1479f-cp38-cp38-linux_aarch64.whl
COPY --from=jetson-pytorch /root/torchvision-0.9.0a0+8fb5838-cp38-cp38-linux_aarch64.whl torch_deps/torchvision-0.9.0a0+8fb5838-cp38-cp38-linux_aarch64.whl
RUN cd torch_deps && \
    ./torch_installer.sh && \
    cd ../ && \
    rm -rf torch_deps

# Install CV Dependencies
RUN pip3 install --no-cache-dir detecto depthai

### ROS Plugins and Dependencies ###

# Install Vimba libraries for avt_vimba_camera
RUN git clone https://github.com/DukeRobotics/camera-driver.git && \
    mv camera-driver/$TARGETPLATFORM/Vimba.tgz Vimba.tgz && \
    tar -xf Vimba.tgz && \
    cd Vimba_4_2/VimbaGigETL && \
    ./Install.sh
RUN cd Vimba_4_2/VimbaPython && \
    pip3 install ./Source[numpy-export,opencv-export]
RUN rm -rf camera-driver && \
    rm Vimba.tgz

# Install Saleae Logic application and add to path
RUN wget -O "logic.zip" "http://downloads.saleae.com/logic/1.2.18/Logic+1.2.18+(64-bit).zip" && \
    unzip logic.zip && \
    mv "Logic 1.2.18 (64-bit)" "Logic" && \
    ln -s /root/docker-build/Logic/Logic /usr/local/bin/Logic

# Build and install OSQP from source
RUN git clone --branch release-0.6.3 --recursive https://github.com/osqp/osqp.git && \
    cd osqp && \
    mkdir build && \
    cd build && \
    cmake -G "Unix Makefiles" .. && \
    cmake --build . --target install && \
    cd ../.. && \
    rm -rf osqp

# Build and install osqp-eigen from source
RUN git clone https://github.com/robotology/osqp-eigen.git && \
    cd osqp-eigen && \
    git checkout 85c37623774c682db396505f0d4ea677040c2557 && \
    mkdir build && \
    cd build && \
    cmake ../ && \
    make && \
    make install && \
    cd ../.. && \
    rm -rf osqp-eigen

# install ros-gz
RUN mkdir -p ~/ros_ign_ws/src && \
    cd ~/ros_ign_ws/src && \
    git clone https://github.com/osrf/ros_ign.git -b noetic && \
    cd ros_ign && \
    git checkout 28a0a75160794451c027b125a6edef1bf919d342 && \
    cd ~/ros_ign_ws && \
    rosdep install -r --from-paths src -i -y --rosdistro noetic
RUN cd ~/ros_ign_ws && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make install"

# Install Arduino CLI for Arduino upload script
# v0.17.0 is required due to issues with adding libraries in the current version v0.18.0
RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=/usr/local/bin sh -s 0.17.0

# Install Arduino cores
RUN arduino-cli core install arduino:megaavr arduino:avr

# Install Arduino libraries
RUN arduino-cli lib update-index
RUN arduino-cli lib install "servo" "DHT11"

# Set ROS_IP to avoid unresolvable hostnames
COPY setup_network.bash /opt/ros/noetic/setup_network.bash
RUN chmod +x /opt/ros/noetic/setup_network.bash

# Set computer type
ENV COMPUTER_TYPE=onboard
RUN echo "COMPUTER_TYPE=onboard" >> /root/.ssh/environment

RUN apt-get clean && \
    # Clear apt caches to reduce image size
    rm -rf /var/lib/apt/lists/*

# Set working directory to codebase
WORKDIR /root/dev/robosub-ros
