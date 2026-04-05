# ROV Onboard

Onboard computer nodes for the underwater ROV. Runs on the Raspberry Pi 5 (Ubuntu 24.04, ROS2 Jazzy).

## Prerequisites

```bash
# ROS2 Jazzy (should already be installed)
sudo apt install ros-jazzy-desktop

# Required ROS2 packages
sudo apt install \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-usb-cam \
    ros-jazzy-image-transport \
    ros-jazzy-compressed-image-transport \
    ros-jazzy-image-transport-plugins \
    ros-jazzy-std-srvs
```

## Setup

```bash
# Build the workspace
cd ~/rov_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rov_cameras --symlink-install

# Or use the build script
./src/rov_cameras/scripts/build.sh

# Clean rebuild if needed
./src/rov_cameras/scripts/build.sh --clean
```

## DDS Configuration

Cyclone DDS with unicast peer discovery (no multicast) for Docker compatibility.

The config file at `~/cyclonedds.xml` must exist with correct peer IPs:
- Onboard (Pi 5): `192.168.1.70`
- Topside: `192.168.1.69`

Two environment variables must be set in every terminal:
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://${HOME}/cyclonedds.xml
```

The startup scripts handle this automatically.

## Running

You need **two terminals** (or SSH sessions) on the Pi 5:

### Terminal 1 — Cameras + Photogrammetry
```bash
./src/rov_cameras/scripts/start_onboard.sh
```

This starts:
- `photogrammetry_node` — HTTP-triggers Pi Zero camera, publishes full-res images
- `pilot_cam_front` — USB camera on `/dev/video0` (640x480 @ 30fps)
- `pilot_cam_down` — USB camera on `/dev/video2` (640x480 @ 30fps)

> **Note:** Pilot cameras will error if USB cameras are not physically connected. The photogrammetry node works independently.

### Terminal 2 — Joystick Translation
```bash
./src/rov_cameras/scripts/start_joy_translator.sh
```

This starts:
- `joy_to_mavlink` — subscribes to `/joy` from topside, translates DS4 inputs to thruster PWM values

Currently in **logging mode** — prints translated values to console. MAVROS RC override output will be added once the ArduPilot flight controller is connected.

## Nodes

| Node | Topic/Service | Type | Direction | Description |
|------|--------------|------|-----------|-------------|
| `photogrammetry_node` | `/photogrammetry/capture` | `Trigger` (srv) | in | Triggers a capture on the Pi Zero |
| `photogrammetry_node` | `/photogrammetry/image` | `CompressedImage` (pub) | out | Full-res JPEG from Pi Zero camera |
| `pilot_cam_front` | `/camera/pilot_front/image_raw` | `Image` (pub) | out | Forward USB camera feed |
| `pilot_cam_down` | `/camera/pilot_down/image_raw` | `Image` (pub) | out | Downward USB camera feed |
| `joy_to_mavlink` | `/joy` | `Joy` (sub) | in | DS4 joystick input from topside |

## Photogrammetry Camera (Pi Zero)

The Pi Zero 2 W runs independently (not ROS). It hosts an HTTP capture server:
- **URL:** `http://192.168.7.2:8080/capture` (over USB ethernet)
- **Status:** `http://192.168.7.2:8080/status`
- Captures 4608x2592 JPEG via Camera Module 3 (IMX708)
- ~0.5 seconds per capture

The `photogrammetry_node` on Pi 5 bridges this into ROS2.

## ArduPilot Connection (Planned)

Connect a Pixhawk-class flight controller via USB to the Pi 5:
- Enumerates as `/dev/ttyACM0`
- MAVROS connects at 115200 baud
- `joy_to_mavlink` will send RC override commands via MAVROS topics

## Debugging

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://${HOME}/cyclonedds.xml
source /opt/ros/jazzy/setup.bash
source ~/rov_ws/install/setup.bash

# Check topics from both machines
ros2 topic list

# Test photogrammetry capture manually
ros2 service call /photogrammetry/capture std_srvs/srv/Trigger

# Monitor joystick translation
ros2 run rov_cameras joy_to_mavlink

# Check Pi Zero camera directly
curl http://192.168.7.2:8080/status
```
