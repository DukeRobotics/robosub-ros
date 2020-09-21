FROM dukerobotics/robosub-ros:base

### Arguments ###
ARG BUILD_DIR_NAME=docker-build
ARG DEBIAN_FRONTEND=noninteractive
ENV WORKDIR=/root/$BUILD_DIR_NAME

# Set the workdir
WORKDIR $WORKDIR

### ROS Installation ###

# install packages
RUN apt-get update && apt-get install --no-install-recommends -q -y \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros1-latest.list

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO melodic
# bootstrap rosdep
RUN rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO

# install ros packages
RUN apt-get update && apt-get install -y \
    ros-melodic-ros-core=1.4.1-0* \
    && rm -rf /var/lib/apt/lists/*

### Basic Setup ###

# Update and install tools
RUN apt-get update && apt-get install -y --no-install-recommends software-properties-common && \
    add-apt-repository ppa:jgmath2000/et && \
    apt-get update && apt-get install -y --no-install-recommends \
    openssh-server mosh et \
    vim \
    curl \
    wget \
    usbutils \
    apt-utils \
    python-catkin-tools \
    ros-melodic-cv-bridge \
    ros-melodic-resource-retriever \
    locales && \
    apt-get clean && \
    # Clear apt caches to reduce image size
    rm -rf /var/lib/apt/lists/*

# Install pip
RUN curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py && \
    python get-pip.py && \
    python -m pip install pip==8.1.1 

# Set up locales
RUN locale-gen en_GB.UTF-8 && locale-gen en_US.UTF-8

# Set a password for root
RUN echo root:robotics | chpasswd

# Allow root to login over ssh
RUN echo "PermitRootLogin yes" >> /etc/ssh/sshd_config

# Generate ssh keys
RUN ssh-keygen -A

### Final Setup ###

# Add catkin ws
RUN mkdir -p /root/dev/robosub-ros

# Copy and set entrypoint script
COPY entrypoint.sh entrypoint.sh
RUN chmod +x entrypoint.sh
ENTRYPOINT $WORKDIR/entrypoint.sh
