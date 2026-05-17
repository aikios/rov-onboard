#!/bin/bash
# ROV Onboard — stop all nodes (native + Docker)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$SCRIPT_DIR/logs"

echo "[stop] Stopping onboard ROV..."

# Stop Docker containers if any are running
cd "$SCRIPT_DIR"
docker compose down 2>/dev/null || true

# Kill native processes by PID files
for pidfile in "$LOG_DIR"/*.pid; do
    if [[ -f "$pidfile" ]]; then
        pid=$(cat "$pidfile")
        kill "$pid" 2>/dev/null && echo "[stop] Killed $(basename "$pidfile" .pid) (PID $pid)"
        rm -f "$pidfile"
    fi
done

# Kill by name in case of foreground mode or missed PIDs
pkill -f 'joy_to_mavlink' 2>/dev/null || true
pkill -f 'mavros_node' 2>/dev/null || true
pkill -f 'photogrammetry' 2>/dev/null || true

# Clean DDS shared memory
rm -f /dev/shm/cyclonedds_* 2>/dev/null || true

echo "[stop] All onboard nodes stopped."
