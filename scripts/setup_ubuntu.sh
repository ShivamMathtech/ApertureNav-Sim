#!/usr/bin/env bash
set -eo pipefail
source /etc/os-release
[[ "$ID" == ubuntu && "$VERSION_ID" == 24.04 ]] || { echo "This script requires Ubuntu 24.04."; exit 1; }
echo "Installing repository development prerequisites"
sudo apt update
sudo apt install -y python3-venv python3-pip python3-numpy python3-yaml python3-opencv git build-essential curl
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  bash "$(dirname "$0")/install_ros_dependencies.sh"
else
  echo "Install ROS 2 Jazzy using docs/windows_setup.md, then run scripts/install_ros_dependencies.sh"
fi
