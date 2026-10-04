#!/usr/bin/env bash
set -eo pipefail
[[ -f /opt/ros/jazzy/setup.bash ]] || { echo "ROS 2 Jazzy is required. See docs/windows_setup.md"; exit 1; }
sudo apt update
sudo apt install -y ros-jazzy-ros-gz ros-jazzy-rviz2 ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-cv-bridge ros-jazzy-tf2-ros ros-jazzy-vision-opencv python3-colcon-common-extensions python3-rosdep python3-pytest
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then sudo rosdep init; fi
rosdep update
