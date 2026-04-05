# ROV Onboard

Onboard computer nodes for the underwater ROV. Runs on the Raspberry Pi 5 (Ubuntu 24.04, ROS2 Jazzy).

## Architecture

`joy_to_mavlink` owns the FC serial port (`/dev/ttyACM0`) and forwards telemetry to MAVROS via UDP. This bypasses a MAVROS 2.14.0 limitation where ManualControl messages published on ROS topics are not forwarded to the FC.

```
joy_to_mavlink ──MANUAL_CONTROL──► /dev/ttyACM0 (FC)
      │
      └──raw telemetry──► UDP:14550 → MAVROS → ROS topics
```

## Nodes

| Node | Description |
|------|-------------|
| `joy_to_mavlink` | Owns FC serial. Receives `/joy` from topside, sends MANUAL_CONTROL to FC. Arms/disarms directly. Forwards FC telemetry to MAVROS. Implements depth hold PID. Sends GCS heartbeat to prevent FC auto-disarm. |
| `photogrammetry_node` | Service `/photogrammetry/capture` — HTTP-triggers Pi Zero camera, publishes full-res JPEG on `/photogrammetry/image`. |

## Prerequisites

```bash
sudo apt install ros-jazzy-desktop ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-usb-cam ros-jazzy-image-transport \
    ros-jazzy-compressed-image-transport ros-jazzy-image-transport-plugins \
    ros-jazzy-std-srvs ros-jazzy-mavros ros-jazzy-mavros-extras
sudo pip3 install --break-system-packages pymavlink
```

## Build

```bash
cd ~/rov_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rov_cameras --symlink-install
```

## Flight Controller Setup

```bash
# Flash ArduSub (from Pi 5)
cd ~/ardupilot_fw
python3 uploader.py ardusub_fmuv3.apj --port /dev/ttyACM0

# Key parameters (set via pymavlink then save+reboot):
#   FRAME_CONFIG = 2 (VECTORED_6DOF, 8 motors)
#   ARMING_CHECK = 0 (disabled for bench testing)
#   BRD_SAFETYOPTION = 0

# USB power cycle (software, no unplug needed):
sudo sh -c "echo 0 > /sys/bus/usb/devices/2-1/authorized"
sleep 3
sudo sh -c "echo 1 > /sys/bus/usb/devices/2-1/authorized"
```

## Running

### Recommended: use the full launcher script from topside
```bash
bash /tmp/launch_rov.sh
```

### Manual (order matters):
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://${HOME}/cyclonedds.xml
source /opt/ros/jazzy/setup.bash
source ~/rov_ws/install/setup.bash

# 1. joy_to_mavlink (must start before MAVROS, owns serial)
ros2 run rov_cameras joy_to_mavlink

# 2. MAVROS (reads UDP:14550, publishes ROS topics)
ros2 launch rov_cameras mavros.launch.py

# 3. Photogrammetry (optional)
ros2 run rov_cameras photogrammetry_node
```

## Photogrammetry Camera (Pi Zero)

Not ROS — standalone HTTP server:
- **URL:** `http://192.168.7.2:8080/capture`
- **Status:** `http://192.168.7.2:8080/status`
- 4608x2592 JPEG, ~0.5s per capture
- `photogrammetry_node` bridges this into ROS2

## Depth Hold

- Reads depth from `/mavros/vfr_hud` (Bar30 sensor via ArduSub)
- PID: kp=1.0, ki=0.1, kd=0.5 (tunable via ROS params)
- Toggle via D-pad left on DS4
- Adjusts setpoint +/-0.25m via D-pad up/down
