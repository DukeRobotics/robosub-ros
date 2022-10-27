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

### Basic Setup ###

# Update and install tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    xauth pciutils \
    libusb-dev libqt5x11extras5 \
    unzip zip util-linux \
    python3-pandas && \
    apt-get clean && \
    # Clear apt caches to reduce image size
    rm -rf /var/lib/apt/lists/*

# Get required python packages
RUN pip3 install --no-cache-dir \
    numpy scipy pymba python-dateutil \
    pyserial dependency-injector \ 
    gputil saleae

# Install PyTorch and Torchvision
COPY torch_installer.sh torch_deps/torch_installer.sh
COPY --from=jetson-pytorch /root/torch-1.8.0a0+5c1479f-cp38-cp38-linux_aarch64.whl torch_deps/torch-1.8.0a0+5c1479f-cp38-cp38-linux_aarch64.whl
COPY --from=jetson-pytorch /root/torchvision-0.9.0a0+8fb5838-cp38-cp38-linux_aarch64.whl torch_deps/torchvision-0.9.0a0+8fb5838-cp38-cp38-linux_aarch64.whl

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

# Install Arduino CLI for Arduino upload script
# v0.17.0 is required due to issues with adding libraries in the current version v0.18.0
RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=/usr/local/bin sh -s 0.17.0

# Set computer type
ENV COMPUTER_TYPE=onboard
RUN echo "COMPUTER_TYPE=onboard" >> /root/.ssh/environment

# Set working directory to codebase
WORKDIR /root/dev/robosub-ros
