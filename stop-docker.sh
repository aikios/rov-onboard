#!/bin/bash
# ROV Onboard — stop all Docker services

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

docker compose down
rm -f /dev/shm/cyclonedds_* 2>/dev/null || true
echo 'Onboard Docker containers stopped.'
