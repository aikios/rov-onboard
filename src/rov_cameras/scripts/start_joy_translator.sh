#!/usr/bin/env bash
# Start the joystick-to-MAVLink translation node.
# Run in a separate terminal alongside start_onboard.sh.
# Usage: ./start_joy_translator.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://${HOME}/cyclonedds.xml

source /opt/ros/jazzy/setup.bash
source "${WS_DIR}/install/setup.bash"

echo "=== ROV Joy→MAVLink ==="
echo "  Subscribing to /joy from topside"
echo ""

ros2 run rov_cameras joy_to_mavlink
