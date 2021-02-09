ARG BASE_IMAGE=dukerobotics/robosub-ros:core
FROM ${BASE_IMAGE}

ARG TARGETPLATFORM

### SSH Setup ###

# Set ssh to use port 2200
RUN echo "Port 2200" >> /etc/ssh/sshd_config

### Basic Setup ###

# Update and install tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    libceres-dev \
    python-numpy python-scipy python-future  \
    python-six python-dateutil python-skimage \
    python-pywt python-pandas \
    ros-melodic-tf \
    ros-melodic-robot-localization  ros-melodic-libg2o \
    ros-melodic-pid ros-melodic-camera-info-manager-py \
    ros-melodic-rosserial ros-melodic-rosserial-arduino && \
    apt-get clean && \
    # Clear apt caches to reduce image size
    rm -rf /var/lib/apt/lists/*

# Get required python packages
RUN pip install Cython
RUN pip install pyserial dependency-injector pillow==6 pymba==0.1 numpy==1.19.5

# Install PyTorch and Torchvision
COPY $TARGETPLATFORM/TorchInstaller.sh TorchInstaller.sh
RUN ./TorchInstaller.sh && \
    rm TorchInstaller.sh

# Install Detecto
RUN pip install --no-cache-dir --no-deps detecto

### ROS Plugins and Dependencies###

# Install Vimba libraries for avt_vimba_camera
RUN git clone https://github.com/muthuArivoli/camera-driver.git && \
    mv camera-driver/$TARGETPLATFORM/Vimba.tgz Vimba.tgz && \
    tar -xf Vimba.tgz && \
    cd Vimba_3_0/VimbaGigETL && \
    ./Install.sh
RUN rm -rf camera-driver && \
    rm Vimba.tgz

# Set computer type
ENV COMPUTER_TYPE=onboard
