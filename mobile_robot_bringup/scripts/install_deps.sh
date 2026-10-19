#!/usr/bin/env bash
# One-shot dependency installer for Ubuntu 24.04 + ROS 2 Jazzy:   bash scripts/install_deps.sh
set -e
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  echo "ROS 2 Jazzy is not installed. Follow https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html first."; exit 1
fi
sudo apt-get update
sudo apt-get install -y \
  ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-rviz2 ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-slam-toolbox ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-map-server \
  ros-jazzy-v4l2-camera ros-jazzy-robot-localization ros-jazzy-rosbag2 ros-jazzy-rosbag2-compression-zstd ros-jazzy-rosbag2-storage-default-plugins \
  python3-scipy python3-matplotlib g++ python3-numpy python3-yaml python3-pytest python3-colcon-common-extensions
echo "Done. Next:  cd ~/dev_ws && colcon build --symlink-install && source install/setup.bash"
