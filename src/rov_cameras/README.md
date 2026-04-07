# ROV Onboard

Onboard computer for the underwater ROV. Runs on Raspberry Pi 5 with Docker.

## Quick Start (Docker)

```bash
git clone git@github.com:aikios/rov-onboard.git && cd rov-onboard

# Edit cyclonedds_onboard.xml — set peer IPs for your network
docker compose up -d
```

## Services

| Container | Description |
|-----------|-------------|
| `joy_to_mavlink` | Owns FC serial, sends MANUAL_CONTROL, forwards telemetry to MAVROS |
| `mavros` | Reads FC telemetry from UDP:14550, publishes ROS topics |
| `photogrammetry` | Polls Pi Zero camera for preview + full-res captures |

## Architecture

```
joy_to_mavlink ──MANUAL_CONTROL──► /dev/ttyACM0 (FC serial)
      │
      └──telemetry──► UDP:14550 → MAVROS → ROS2 topics
```

## Flight Controller

- ArduSub 4.5.7, FRAME_CONFIG=2 (VECTORED_6DOF, 8 motors)
- Flash: `python3 ~/ardupilot_fw/uploader.py ardusub_fmuv3.apj --port /dev/ttyACM0`
- USB power cycle: `echo 0/1 > /sys/bus/usb/devices/2-1/authorized`

## Pi Zero Camera

HTTP server at 192.168.7.2:8080 — `/preview` (640x480) and `/capture` (full-res)

## DDS Config

Edit `cyclonedds_onboard.xml` with your network IPs.
