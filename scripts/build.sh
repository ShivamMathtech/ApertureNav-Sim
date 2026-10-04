#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
[[ -f /opt/ros/jazzy/setup.bash ]] || { echo "Install ROS 2 Jazzy first; see docs/windows_setup.md"; exit 1; }
source /opt/ros/jazzy/setup.bash
python3 scripts/sync_ros_core.py
cd drone_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
