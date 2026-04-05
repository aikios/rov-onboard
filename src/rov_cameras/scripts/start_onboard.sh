#!/usr/bin/env bash
# Start all onboard ROV nodes on Pi 5.
# Usage: ./start_onboard.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"

# DDS config
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://${HOME}/cyclonedds.xml

# Source ROS2 and workspace
source /opt/ros/jazzy/setup.bash
source "${WS_DIR}/install/setup.bash"

echo "=== ROV Onboard (Pi 5) ==="
echo "  DDS:       Cyclone (unicast peers)"
echo "  Workspace: ${WS_DIR}"
echo ""

# Launch cameras + photogrammetry
ros2 launch rov_cameras cameras.launch.py
