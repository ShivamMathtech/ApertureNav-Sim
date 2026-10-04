#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash
[[ -f drone_ws/install/setup.bash ]] || { echo "Run bash scripts/build.sh first"; exit 1; }
source drone_ws/install/setup.bash
exec ros2 launch drone_bringup full_simulation.launch.py "config:=$(pwd)/config/default.yaml" "results:=$(pwd)/experiments/results" "$@"
