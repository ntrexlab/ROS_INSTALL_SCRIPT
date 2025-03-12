#!/bin/bash
# Apache License 2.0
# Copyright (c) 2025, NTREX CO., LTD.

set -e  # Exit immediately if a command exits with a non-zero status

# Update and upgrade system packages
sudo apt-get update && sudo apt-get upgrade -y

# Download ROS 2 Jazzy desktop package
wget https://s3.ap-northeast-1.wasabisys.com/download-raw/dpkg/ros2-desktop/debian/bookworm/ros-jazzy-desktop-0.3.2_20240525_arm64.deb

# Set appropriate permissions
sudo chmod 644 ./ros-jazzy-desktop-0.3.2_20240525_arm64.deb

# Install ROS 2 Jazzy desktop package
sudo apt install -y ./ros-jazzy-desktop-0.3.2_20240525_arm64.deb

# Create workspaces
mkdir -p ~/jazzy_ws/src
mkdir -p ~/colcon_ws/src

# Add helpful aliases to ~/.bashrc
echo "alias nb='nano ~/.bashrc'" >> ~/.bashrc
echo "alias sb='source ~/.bashrc'" >> ~/.bashrc
echo "alias cb='cd ~/colcon_ws && colcon build --symlink-install && source ~/.bashrc'" >> ~/.bashrc

# Source ROS Jazzy setup and workspace setup
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/jazzy_ws/install/local_setup.bash" >> ~/.bashrc
echo "source ~/colcon_ws/install/local_setup.bash" >> ~/.bashrc

# Apply changes
source ~/.bashrc

# Install additional dependencies
sudo apt install -y python3-rosinstall-generator build-essential cmake python3-argcomplete python3-rosinstall python3-pip

# Install Colcon extensions
sudo pip3 install --break-system-packages colcon-common-extensions colcon-ros

# Completion message
echo "ROS 2 Jazzy installation completed successfully."
