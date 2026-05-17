#!/bin/bash
# start.sh — Launch all onboard ROV nodes natively
# Usage: ./start.sh        (foreground logs)
#        ./start.sh -d     (background/daemon mode)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$SCRIPT_DIR/logs"
mkdir -p "$LOG_DIR"

# --- Cleanup stale DDS state ---
echo "[start] Cleaning up stale DDS shared memory..."
rm -f /dev/shm/cyclonedds_* 2>/dev/null || true

# --- Kill any existing ROV nodes ---
echo "[start] Killing stale ROV processes..."
pkill -f 'joy_to_mavlink' 2>/dev/null || true
pkill -f 'mavros_node' 2>/dev/null || true
pkill -f 'photogrammetry' 2>/dev/null || true
sleep 1

# --- Environment ---
source /opt/ros/jazzy/setup.bash
source "$SCRIPT_DIR/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$SCRIPT_DIR/cyclonedds_onboard.xml

DAEMON=false
if [[ "$1" == "-d" ]]; then
    DAEMON=true
fi

launch_node() {
    local name="$1"
    shift
    if $DAEMON; then
        echo "[start] Launching $name (background, log: $LOG_DIR/$name.log)"
        nohup "$@" > "$LOG_DIR/$name.log" 2>&1 &
        echo $! > "$LOG_DIR/$name.pid"
    else
        echo "[start] Launching $name"
        "$@" &
    fi
}

# --- 1. joy_to_mavlink (must start first — owns serial, DDS discovery order) ---
launch_node joy_to_mavlink ros2 run rov_flight joy_to_mavlink
sleep 2

# --- 2. MAVROS (reads FC telemetry from UDP:14550) ---
launch_node mavros ros2 launch rov_flight mavros.launch.py
sleep 1

# --- 3. Photogrammetry node (Pi Zero camera bridge) ---
launch_node photogrammetry ros2 run rov_photogrammetry node

echo "[start] All onboard nodes launched."

if ! $DAEMON; then
    echo "[start] Press Ctrl+C to stop all nodes."
    trap 'echo "[start] Stopping..."; kill 0; exit' SIGINT SIGTERM
    wait
fi
