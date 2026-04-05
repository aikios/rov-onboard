#!/usr/bin/env bash
# Build the onboard ROS2 workspace.
# Usage: ./build.sh [--clean]
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"

source /opt/ros/jazzy/setup.bash

if [ "${1:-}" = "--clean" ]; then
    echo "Clean build..."
    rm -rf "${WS_DIR}/build" "${WS_DIR}/install" "${WS_DIR}/log"
fi

cd "${WS_DIR}"
colcon build --packages-select rov_cameras --symlink-install
echo "Build complete. Source with: source ${WS_DIR}/install/setup.bash"
