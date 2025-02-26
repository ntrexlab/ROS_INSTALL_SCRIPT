#!/bin/bash
# Apache License 2.0
# Copyright (c) 2024, NTREX CO., LTD.

echo ""
echo "[Note] Target OS version  >>> Raspberry Pi OS Bookworm (arm64)"
echo "[Note] Target ROS version >>> ROS 2 Jazzy"
echo "[Note] Colcon workspace   >>> $HOME/colcon_ws"
echo ""
echo "PRESS [ENTER] TO CONTINUE THE INSTALLATION"
echo "IF YOU WANT TO CANCEL, PRESS [CTRL] + [C]"
read

echo "[Set the target OS, ROS version, and name of colcon workspace]"
name_os_version=${name_os_version:="bookworm"}
name_ros_version=${name_ros_version:="jazzy"}
name_colcon_workspace=${name_colcon_workspace:="colcon_ws"}

echo "[Update the package lists and upgrade them]"
sudo apt update -y && sudo apt upgrade -y

echo "[Install build environment, chrony, ntpdate, and set the time sync]"
sudo apt install -y chrony ntpdate
sudo ntpdate ntp.ubuntu.com

echo "[Add the ROS2 repository]"
sudo apt install -y curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu bookworm main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "[Update the package lists and upgrade them]"
sudo apt update -y && sudo apt upgrade -y

echo "[Install the bootstrap dependencies]"
sudo apt install -y build-essential cmake python3-argcomplete python3-colcon-common-extensions python3-rosdep python3-rosinstall-generator python3-vcstool python3-rosinstall

echo "[Initialize rosdep]"
sudo rosdep init || echo "rosdep already initialized"
rosdep update

echo "[Create a colcon workspace and fetch the core packages]"
mkdir -p ~/$name_colcon_workspace/src
cd ~/$name_colcon_workspace
vcs import src < https://raw.githubusercontent.com/ros2/ros2/jazzy/ros2.repos

echo "[Resolve Dependencies]"
rosdep install -y --from-paths src --ignore-src --rosdistro $name_ros_version -r --os=debian:$name_os_version

echo "[Build the colcon workspace]"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

echo "[Environment setup]"
source ~/$name_colcon_workspace/install/setup.sh

echo "[Set the ROS environment]"
sh -c "echo \"alias nb='nano ~/.bashrc'\" >> ~/.bashrc"
sh -c "echo \"alias eb='nano ~/.bashrc'\" >> ~/.bashrc"
sh -c "echo \"alias sb='source ~/.bashrc'\" >> ~/.bashrc"
sh -c "echo \"alias gs='git status'\" >> ~/.bashrc"
sh -c "echo \"alias gp='git pull'\" >> ~/.bashrc"

sh -c "echo \"alias cw='cd ~/$name_colcon_workspace'\" >> ~/.bashrc"
sh -c "echo \"alias cs='cd ~/$name_colcon_workspace/src'\" >> ~/.bashrc"
sh -c "echo \"alias cb='cd ~/$name_colcon_workspace && colcon build --symlink-install && source ~/.bashrc'\" >> ~/.bashrc"

sh -c "echo \"source /opt/ros/$name_ros_version/setup.bash\" >> ~/.bashrc"
sh -c "echo \"source ~/$name_colcon_workspace/install/local_setup.bash\" >> ~/.bashrc"

source ~/.bashrc

echo "[Complete!!!]"
exit 0
