# ROV Onboard

Onboard computer for the underwater ROV. Runs on Raspberry Pi 5 (Ubuntu 24.04, aarch64) with Docker.

## Prerequisites

- Raspberry Pi 5 with Ubuntu 24.04
- Docker + Docker Compose (`curl -fsSL https://get.docker.com | sudo sh`)
- Pixhawk1 flight controller connected via USB (`/dev/ttyACM0`)
- Pi Zero 2 W with camera server at `192.168.7.2:8080` (USB ethernet)
- Network access to topside machine

## Production Deployment

```bash
# 1. Clone
git clone git@github.com:aikios/rov-onboard.git
cd rov-onboard

# 2. Configure network — edit cyclonedds_onboard.xml:
#    Set <Peer address="..."/> to:
#      - This Pi 5's IP
#      - Topside machine's IP
nano cyclonedds_onboard.xml

# 3. Start
./start.sh

# 4. Stop
./stop.sh
```

## ROS2 Packages

| Package | Container | Purpose |
|---------|-----------|---------|
| `rov_flight` | `joy_to_mavlink` | Owns FC serial, sends MANUAL_CONTROL, forwards telemetry via UDP:14550, arming + depth hold PID |
| `rov_flight` | `mavros` | Reads FC telemetry from UDP:14550, publishes ROS topics |
| `rov_photogrammetry` | `photogrammetry` | Polls Pi Zero camera for 640x480 preview at 2Hz, full-res captures |

## Architecture

```
/joy (from topside) → joy_to_mavlink → /dev/ttyACM0 → Pixhawk FC
                           │
                      UDP:14550 → mavros → /mavros/* topics → topside
Pi Zero (192.168.7.2) → photogrammetry → /photogrammetry/* → topside
```

MAVROS 2.14.0 does not forward ManualControl from ROS topics to the FC, so `joy_to_mavlink` owns the serial port directly.

## Flight Controller Setup

Only needed once (run natively, not in Docker):

```bash
# Install pymavlink
sudo pip3 install --break-system-packages pymavlink

# Flash ArduSub firmware
cd ~/ardupilot_fw
wget https://firmware.ardupilot.org/Sub/stable/fmuv3/ardusub.apj
python3 uploader.py ardusub.apj --port /dev/ttyACM0

# Set parameters (via pymavlink script):
#   FRAME_CONFIG = 2  (VECTORED_6DOF, 8 motors)
#   ARMING_CHECK = 0  (disable for bench testing)
#   BRD_SAFETYOPTION = 0

# Software USB power cycle (no physical unplug):
sudo sh -c "echo 0 > /sys/bus/usb/devices/2-1/authorized"
sleep 3
sudo sh -c "echo 1 > /sys/bus/usb/devices/2-1/authorized"
```

## Pi Zero Camera Server

The Pi Zero 2 W runs independently (not Docker, not ROS). HTTP server at `192.168.7.2:8080`:

| Endpoint | Description |
|----------|-------------|
| `GET /preview` | 640x480 JPEG for live preview |
| `GET /capture` | 4608x2592 full-res JPEG |
| `GET /status` | Camera status JSON |

Setup docs: see `pi-webcam-gadget-setup.md` in the main project.

## Native Development

```bash
sudo apt install ros-jazzy-desktop ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-mavros ros-jazzy-mavros-extras ros-jazzy-std-srvs
sudo pip3 install --break-system-packages pymavlink

cd ~/rov_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml

ros2 run rov_flight joy_to_mavlink &      # start FIRST (owns serial)
ros2 launch rov_flight mavros.launch.py &
ros2 run rov_photogrammetry node &
```
