#!/bin/bash
# Apache License 2.0
# Copyright (c) 2025, NTREX CO., LTD.

echo ""
echo "[Note] OS version  >>> Ubuntu 24.04 (Noble Numbat)"
echo "[Note] Target ROS version >>> ROS 2 Jazzy"
echo "[Note] Colcon workspace   >>> $HOME/colcon_ws"
echo ""
echo "PRESS [ENTER] TO CONTINUE THE INSTALLATION"
echo "IF YOU WANT TO CANCEL, PRESS [CTRL] + [C]"
read

echo "[Set the target ROS version and name of colcon workspace]"
name_ros_version=${name_ros_version:="jazzy"}
name_colcon_workspace=${name_colcon_workspace:="colcon_ws"}

echo "[Set Locale]"
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo "[Setup Sources]"
sudo apt update && sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo add-apt-repository universe
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "[Install ROS 2 Jazzy (Development Setup)]"
sudo apt update && sudo apt install -y \
  build-essential cmake git python3-pip python3-rosdep python3-colcon-common-extensions python3-vcstool

echo "[Initialize rosdep]"
sudo rosdep init || echo "rosdep already initialized"
rosdep update

echo "[Make the colcon workspace]"
mkdir -p $HOME/$name_colcon_workspace/src
cd $HOME/$name_colcon_workspace

echo "[Get ROS 2 Jazzy source code]"
vcs import src < https://raw.githubusercontent.com/ros2/ros2/jazzy/ros2.repos
rosdep install --from-paths src --ignore-src --rosdistro $name_ros_version -y

echo "[Build ROS 2 Jazzy]"
colcon build --symlink-install

echo "[Set the ROS environment]"
sh -c "echo \"alias nb='nano ~/.bashrc'\" >> ~/.bashrc"
sh -c "echo \"alias sb='source ~/.bashrc'\" >> ~/.bashrc"
sh -c "echo \"alias gs='git status'\" >> ~/.bashrc"
sh -c "echo \"alias gp='git pull'\" >> ~/.bashrc"

sh -c "echo \"alias cw='cd ~/$name_colcon_workspace'\" >> ~/.bashrc"
sh -c "echo \"alias cs='cd ~/$name_colcon_workspace/src'\" >> ~/.bashrc"
sh -c "echo \"alias cb='cd ~/$name_colcon_workspace && colcon build --symlink-install && source ~/.bashrc'\" >> ~/.bashrc"

sh -c "echo \"source /opt/ros/$name_ros_version/setup.bash\" >> ~/.bashrc"
sh -c "echo \"source ~/$name_colcon_workspace/install/local_setup.bash\" >> ~/.bashrc"

echo "[Complete!!!]"

exec bash

exit 0
