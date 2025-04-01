# Start with Ubuntu 22.04 (Jammy Jellyfish)
FROM ubuntu:22.04

# Avoid interactive dialog during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Set shell to bash
SHELL ["/bin/bash", "-c"]

# Update package list and install basic utilities
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# Add ROS2 repository
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Add ROS2 GPG key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Install ROS2 Humble and development tools
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Source ROS2 setup in .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc