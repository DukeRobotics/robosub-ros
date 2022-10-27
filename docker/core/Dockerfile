FROM ros:humble

### Arguments ###
ARG BUILD_DIR_NAME=docker-build
ARG DEBIAN_FRONTEND=noninteractive
ENV WORKDIR=/root/$BUILD_DIR_NAME

# Set the workdir
WORKDIR $WORKDIR

### Basic Setup ###

# Update and install tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    openssh-server xvfb \
    vim \
    curl wget \
    usbutils apt-utils \
    ros-humble-resource-retriever \
    python-dev-is-python3 \
    python3-pip \
    python3-osrf-pycommon \
    git \
    locales && \
    apt-get clean && \
    # Clear apt caches to reduce image size
    rm -rf /var/lib/apt/lists/*

# Set up locales
RUN locale-gen en_GB.UTF-8 && locale-gen en_US.UTF-8

# Set a password for root
RUN echo root:robotics | chpasswd

# Allow root to login over ssh and forward graphics over ssh
# Allow for environment variables to be set in ssh
RUN echo "PermitRootLogin yes" >> /etc/ssh/sshd_config && \
    echo "X11UseLocalhost no" >> /etc/ssh/sshd_config && \
    echo "PermitUserEnvironment yes" >> /etc/ssh/sshd_config && \
    mkdir -p /root/.ssh && \
    touch /root/.ssh/environment

COPY ssh_entry.sh /root/.bash_profile

### Final Setup ###

# Install common pip packages
RUN pip3 install autopep8 psutil
# Install correct setuptools version
RUN pip install setuptools==58.2.0

RUN mkdir -p /root/dev/robosub-ros

# Copy and set entrypoint script
COPY entrypoint.sh entrypoint.sh
RUN chmod +x entrypoint.sh
ENTRYPOINT $WORKDIR/entrypoint.sh
