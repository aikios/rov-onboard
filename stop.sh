#!/bin/bash
# ROV Onboard — stop all services
set -e
docker compose down
rm -f /dev/shm/cyclonedds_* /dev/shm/iox_* 2>/dev/null
echo "Onboard stopped."
