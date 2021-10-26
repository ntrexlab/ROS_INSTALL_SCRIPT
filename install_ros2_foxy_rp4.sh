#!/bin/bash
# Apache License 2.0
# Copyright (c) 2021, NTREX CO., LTD.

echo ""
echo "[Note] Target OS version  >>> Raspberry Pi OS Buster for the Raspberry Pi"
echo "[Note] Target ROS version >>> ROS 2 Foxy Fitzroy"
echo "[Note] Colcon workspace   >>> $HOME/colcon_ws"
echo ""
echo "PRESS [ENTER] TO CONTINUE THE INSTALLATION"
echo "IF YOU WANT TO CANCEL, PRESS [CTRL] + [C]"
read

echo "[Set the target OS, ROS version and name of colcon workspace]"
name_os_version=${name_os_version:="buster"}
name_ros_version=${name_ros_version:="foxy"}
name_colcon_workspace=${name_colcon_workspace:="colcon_ws"}

echo "[Update the package lists and upgrade them]"
sudo apt-get update -y
sudo apt-get upgrade -y

echo "[Install build environment, the chrony, ntpdate and set the ntpdate]"
sudo apt-get install -y chrony ntpdate
sudo ntpdate ntp.ubuntu.com

echo "[Add the ROS2 repository]"
sudo apt update && sudo apt install -y curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "[Update the package lists and upgrade them]"
sudo apt-get update -y
sudo apt-get upgrade -y

echo "[Install the bootstrap dependencies]"
sudo apt install -y build-essential cmake python3-argcomplete python3-colcon-common-extensions python3-rosdep python3-rosinstall-generator python3-vcstool python3-rosinstall

echo "[Initialize rosdep]"
sudo sh -c "rosdep init"
rosdep update

echo "[Create a catkin Workspace and fetch the core packages]"
mkdir ~/ros_colcon_ws
cd ~/ros_colcon_ws
rosinstall_generator ros_base nav_msgs cv_bridge example_interfaces demo_nodes_cpp demo_nodes_py joint_state_publisher --rosdistro foxy --deps --wet-only --tar > foxy-ros_base-wet.rosinstall
vcs import src foxy-ros_base-wet.rosinstall

echo "[Resolve Dependencies]"
rosdep install -y --from-paths src --ignore-src --rosdistro foxy -r --os=debian:$name_os_version

echo "[Build the colcon workspace]"
colcon build --symlink-install --cmake-args -DCMAKE_SHARED_LINKER_FLAGS='-latomic -lpython3.7m' -DCMAKE_EXE_LINKER_FLAGS='-latomic -lpython3.7m' -DCMAKE_BUILD_TYPE=RelWithDebInfo --no-warn-unused-cli

echo "[Environment setup]"
source ~/ros_colcon_ws/install/setup.sh

echo "[Make the colcon workspace and test the colcon build]"
mkdir -p $HOME/$name_colcon_workspace/src
cd $HOME/$name_colcon_workspace
colcon build --symlink-install

echo "[Set the ROS evironment]"
sh -c "echo \"alias nb='nano ~/.bashrc'\" >> ~/.bashrc"
sh -c "echo \"alias eb='nano ~/.bashrc'\" >> ~/.bashrc"
sh -c "echo \"alias sb='source ~/.bashrc'\" >> ~/.bashrc"
sh -c "echo \"alias gs='git status'\" >> ~/.bashrc"
sh -c "echo \"alias gp='git pull'\" >> ~/.bashrc"

sh -c "echo \"alias cw='cd ~/$name_colcon_workspace'\" >> ~/.bashrc"
sh -c "echo \"alias cs='cd ~/$name_colcon_workspace/src'\" >> ~/.bashrc"
sh -c "echo \"alias cb='cd ~/$name_colcon_workspace && colcon build --symlink-install && source ~/.bashrc'\" >> ~/.bashrc"

sh -c "echo \"source ~/ros_colcon_ws/install/setup.bash\" >> ~/.bashrc"
sh -c "echo \"source ~/$name_colcon_workspace/install/local_setup.bash\" >> ~/.bashrc"

source $HOME/.bashrc

echo "[Complete!!!]"
exit
