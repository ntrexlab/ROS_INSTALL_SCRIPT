#!/bin/bash
# Apache License 2.0
# Copyright (c) 2025, NTREX CO., LTD.


# Update and upgrade the system
sudo apt update && sudo apt upgrade -y

# Install essential packages
sudo apt install -y net-tools openssh-server git curl locales software-properties-common

# Generate locale settings
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add universe repository
sudo add-apt-repository universe

# Add ROS repository key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the ROS2 repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package list again
sudo apt update

# Install ROS development tools
sudo apt install -y ros-dev-tools

# Install ROS Jazzy Desktop version
sudo apt install -y ros-jazzy-desktop

mkdir -p ~/colcon_ws/src

# Add helpful aliases to ~/.bashrc
echo "alias nb='nano ~/.bashrc'" >> ~/.bashrc
echo "alias sb='source ~/.bashrc'" >> ~/.bashrc
echo "alias cb='cd ~/colcon_ws && colcon build --symlink-install && source ~/.bashrc'" >> ~/.bashrc

# Source ROS Jazzy setup and workspace setup
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/colcon_ws/install/local_setup.bash" >> ~/.bashrc

# Source the new .bashrc to apply changes
source ~/.bashrc

echo "ROS 2 Jazzy Desktop installation completed successfully!"
