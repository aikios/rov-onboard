#!/bin/bash
# ROV Onboard — start all services
# Usage: ./start.sh
set -e

echo "=== ROV Onboard ==="

# Clean stale DDS shared memory
rm -f /dev/shm/cyclonedds_* /dev/shm/iox_* 2>/dev/null

# Ensure serial port exists
if [ ! -e /dev/ttyACM0 ]; then
    echo "WARNING: /dev/ttyACM0 not found — is the flight controller connected?"
    echo "Trying USB power cycle..."
    DEVPATH=$(find /sys/bus/usb/devices -maxdepth 1 -name "[0-9]*" 2>/dev/null | while read d; do
        [ -f "$d/idVendor" ] && grep -q 1209 "$d/idVendor" 2>/dev/null && echo "$d" && break
    done)
    if [ -n "$DEVPATH" ]; then
        sudo sh -c "echo 0 > $DEVPATH/authorized"; sleep 3
        sudo sh -c "echo 1 > $DEVPATH/authorized"; sleep 5
    fi
    [ -e /dev/ttyACM0 ] && echo "FC recovered" || echo "FC still not found, continuing anyway..."
fi

# Start
docker compose up -d
sleep 5

# Verify
echo ""
docker compose ps --format "table {{.Name}}\t{{.Status}}"
echo ""
docker logs rov_ws-joy_to_mavlink-1 2>&1 | tail -1
docker logs rov_ws-photogrammetry-1 2>&1 | tail -1
