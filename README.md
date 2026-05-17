# ROV Onboard

Onboard computer software for the underwater ROV. Runs on Raspberry Pi 5 (Ubuntu 24.04 aarch64).

## Hardware

| Component | Details |
|-----------|---------|
| Computer | Raspberry Pi 5, Ubuntu 24.04 aarch64 |
| Flight Controller | Pixhawk1 (fmuv3, 2MB flash), USB → `/dev/ttyACM0` |
| Firmware | ArduSub 4.5.7, VECTORED_6DOF frame (8 thrusters on SERVO1–8) |
| Depth Sensor | Blue Robotics Bar30 on I2C |
| Photogrammetry Camera | Pi Zero 2 W + Camera Module 3 via USB gadget (`192.168.7.2:8080`) |
| Topside Link | Ethernet tether to `192.168.1.69` |

## Quick Start

### Native (recommended for development)

```bash
git clone git@github.com:aikios/rov-onboard.git ~/rov_ws
cd ~/rov_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/rov_ws/cyclonedds_onboard.xml

./start.sh          # foreground (Ctrl+C to stop)
./start.sh -d       # daemon mode (logs in ./logs/)
./stop.sh           # stop everything
```

### Docker

```bash
./start-docker.sh   # builds image if needed, starts all containers
./stop-docker.sh    # stops and removes containers
```

## ROS2 Packages

### `rov_flight` — Flight control and joystick translation

**Node:** `joy_to_mavlink`

Owns the FC serial port directly and runs the full control loop:

- Subscribes `/joy` from topside, applies deadzone/noise filtering
- Sends `MANUAL_CONTROL` MAVLink messages to ArduSub at 20 Hz
- Forwards raw FC telemetry bytes to `UDP:14550` so MAVROS can read them (read-only bridge)
- **Depth hold:** Square button toggles PID depth hold; L1/R1 adjust setpoint in 0.25 m steps
- **PID auto-tuner:** Åström-Hägglund relay method — triggers via `/rov/autotune_trigger`, runs 4 oscillation cycles, applies Ziegler-Nichols gains automatically
- Arm/disarm via Options button; photogrammetry capture via Circle button
- Publishes `/rov/depth_setpoint`, `/rov/depth_current`, `/rov/depth_hold_active`, `/rov/pid_status`, `/rov/fc_heartbeat`

**Node:** `mavros` (via `ros2 launch rov_flight mavros.launch.py`)

Reads FC telemetry from `UDP:14550`, publishes standard `/mavros/*` topics consumed by the topside dashboard.

### `rov_photogrammetry` — Camera bridge

**Node:** `photogrammetry_node`

Bridges the Pi Zero camera server to ROS:

- Polls `http://192.168.7.2:8080/preview` at 2 Hz → publishes `sensor_msgs/CompressedImage` on `/photogrammetry/preview`
- On `/photogrammetry/capture` service call → fetches `http://192.168.7.2:8080/capture?quality=95` → publishes full-res JPEG on `/photogrammetry/image`

## ROS Topics Published

| Topic | Type | Description |
|-------|------|-------------|
| `/rov/fc_heartbeat` | `std_msgs/Bool` | FC connection alive |
| `/rov/depth_current` | `std_msgs/Float64` | Depth in metres (positive = deeper) |
| `/rov/depth_setpoint` | `std_msgs/Float64` | Active depth hold target |
| `/rov/depth_hold_active` | `std_msgs/Bool` | Depth hold engaged |
| `/rov/pid_status` | `std_msgs/String` | Human-readable PID/tuner state |
| `/photogrammetry/preview` | `sensor_msgs/CompressedImage` | 640×480 preview at ~2 fps |
| `/photogrammetry/image` | `sensor_msgs/CompressedImage` | Full-res capture (4608×2592) |
| `/mavros/state` | `mavros_msgs/State` | FC armed/mode/connected |
| `/mavros/vfr_hud` | `mavros_msgs/VfrHud` | Depth, heading, speed |
| `/mavros/battery` | `sensor_msgs/BatteryState` | Battery voltage |

## ROS Services

| Service | Type | Description |
|---------|------|-------------|
| `/photogrammetry/capture` | `std_srvs/Trigger` | Trigger a full-res still |

## Controller Mapping

| Button | Action |
|--------|--------|
| Left stick Y / X | Surge / Sway |
| Right stick Y / X | Heave / Yaw (manual mode) |
| Square | Toggle depth hold |
| L1 / R1 | Depth setpoint −0.25 m / +0.25 m |
| Circle | Photogrammetry capture |
| Options | Arm / Disarm |

## Network & DDS

CycloneDDS with unicast peer discovery (no multicast). Edit `cyclonedds_onboard.xml` to set peer IPs if your network changes:

```xml
<Peer address="192.168.1.69"/>  <!-- topside -->
<Peer address="192.168.1.70"/>  <!-- this Pi (self) -->
```

All containers and native nodes use `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.

## Flight Controller Setup

One-time setup (run natively, not in Docker):

```bash
# Flash ArduSub firmware
cd ~/ardupilot_fw
python3 uploader.py ardusub_fmuv3.apj --port /dev/ttyACM0

# Key ArduSub parameters
# FRAME_CONFIG = 2  (VECTORED_6DOF, 8 thrusters)
# ARMING_CHECK = 0  (disable for bench testing without thrusters)
# BRD_SAFETYOPTION = 0

# Software USB power cycle if FC stops responding
sudo sh -c "echo 0 > /sys/bus/usb/devices/2-1/authorized" && sleep 3
sudo sh -c "echo 1 > /sys/bus/usb/devices/2-1/authorized"
```

## Pi Zero Camera Server

The Pi Zero 2 W runs a lightweight libcamera HTTP server (not ROS). It connects to the Pi 5 via USB gadget ethernet at `192.168.7.2`.

| Endpoint | Description |
|----------|-------------|
| `GET /preview` | 640×480 JPEG (~2fps capable) |
| `GET /capture?quality=95` | 4608×2592 full-res JPEG |
| `GET /status` | Camera status JSON |

Setup guide: `pi-webcam-gadget-setup.md` on the Pi 5 at `~/pi-webcam-gadget-setup.md`.

## Dependencies

Installed by Dockerfile / `start.sh` prerequisites:

- `ros-jazzy-rmw-cyclonedds-cpp`
- `ros-jazzy-mavros`, `ros-jazzy-mavros-extras`, `ros-jazzy-mavros-msgs`
- `ros-jazzy-std-srvs`
- `python3-pymavlink`

## Related Repo

Topside pilot station: [`aikios/rov-topside`](https://github.com/aikios/rov-topside)
