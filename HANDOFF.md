# ROV Project Handoff

## Overview

This is an underwater ROV system built for photogrammetry and manual piloting. It consists of two computers communicating over a local network:

- **Onboard (Raspberry Pi 5)**: `hydromeda@192.168.1.70` — runs flight control and camera interfaces
- **Topside (Ubuntu x86_64 laptop)**: `ahmad@192.168.1.69` — runs joystick driver, web dashboard, and image saving

Both sides communicate via ROS2 Jazzy over CycloneDDS with unicast peer discovery (no multicast, required for Docker host networking).

---

## Hardware

| Component | Details |
|-----------|---------|
| ROV frame | BlueROV2 HEAVY, VECTORED_6DOF configuration |
| Flight controller | Pixhawk1 fmuv3 running ArduSub 4.5.7 |
| Onboard computer | Raspberry Pi 5 (ARM64, Ubuntu 24.04) |
| Topside computer | Ubuntu 24.04 x86_64 laptop |
| Controller | DualShock 4 (USB, read via evdev) |
| Camera | Pi Zero 2W running libcamera HTTP server at `192.168.7.2:8080` (USB gadget ethernet on Pi 5) |

---

## Repositories

| Repo | Machine | Path |
|------|---------|------|
| `aikios/rov-onboard` | Pi 5 | `~/rov_ws/` |
| `aikios/rov-topside` | Laptop | `~/rov_topside_ws/` |

---

## Architecture

### Onboard (`rov-onboard`)

**ROS2 packages:**

- **`rov_flight`** — Core flight package
  - `joy_to_mavlink.py`: Owns `/dev/ttyACM0` serial to Pixhawk. Receives `/joy` from topside, converts to MAVLink `MANUAL_CONTROL` messages. Implements PID depth hold with Åström–Hägglund relay auto-tuner. Handles arm/disarm and photogrammetry capture trigger.
  - `mavros.launch.py`: Launches MAVROS for read-only telemetry via UDP:14550 (MAVROS does **not** forward MANUAL_CONTROL — that goes direct serial).

- **`rov_photogrammetry`** — Camera interface
  - `node.py`: Polls Pi Zero HTTP server at `192.168.7.2:8080`, publishes `/photogrammetry/preview` (JPEG) and `/photogrammetry/image` (full resolution) topics.

**DDS config:** `cyclonedds_onboard.xml` — unicast peers: topside (192.168.1.69) and self (192.168.1.70).

### Topside (`rov-topside`)

**ROS2 packages:**

- **`rov_joystick`** — Controller driver
  - `joy_publisher.py`: Reads DualShock 4 via evdev (not ros2-joy — DS4 has known issues with it). Publishes `/joy` at 20 Hz.

- **`rov_dashboard`** — Web UI
  - `server.py`: HTTP server on `:8080` + WebSocket on `:9090`. Pushes ROV state at 10 Hz to browser clients.

- **`rov_photogrammetry`** — Image saving
  - `saver.py`: Subscribes to `/photogrammetry/image`, saves timestamped JPEGs locally.

**DDS config:** `cyclonedds_topside.xml` — unicast peers: onboard (192.168.1.70) and self (192.168.1.69).

---

## Running the System

### Option A: Docker (recommended for deployment)

**Onboard (Pi 5):**
```bash
cd ~/rov_ws
./start-docker.sh    # starts joy_to_mavlink, mavros, photogrammetry_node
./stop-docker.sh     # teardown
```

**Topside (laptop):**
```bash
cd ~/rov_topside_ws
./start-docker.sh    # starts joy_publisher, web_dashboard, photogrammetry_saver
./stop-docker.sh     # teardown
```

### Option B: Native (for development)

```bash
# On Pi:
cd ~/rov_ws && ./start.sh [-d]   # -d for background daemon mode

# On laptop:
cd ~/rov_topside_ws && ./start.sh [-d]
```

Use `stop.sh` to stop either.

---

## Building Docker Images

**Onboard (ARM64 — build on Pi):**
```bash
cd ~/rov_ws
docker build -t rov-onboard .
```

**Topside (x86_64 — build on laptop):**
```bash
cd ~/rov_topside_ws
docker build -t rov-topside .
```

Both images are based on `ros:jazzy-ros-base` and run `colcon build --symlink-install` at build time.

---

## Key Design Decisions

### Why evdev instead of ros2-joy?
DualShock 4 has axis/button mapping issues with the standard ros2-joy driver. Evdev gives reliable raw access.

### Why MANUAL_CONTROL direct serial, not through MAVROS?
MAVROS does not forward `MANUAL_CONTROL` messages from ROS topics to the FC — it only relays incoming MAVLink telemetry. `joy_to_mavlink.py` owns the serial port directly and sends MAVLink frames itself.

### Why CycloneDDS unicast?
Docker with `network_mode: host` and multi-machine setups often have multicast blocked or unreliable. Explicit unicast peer lists in `cyclonedds_*.xml` give deterministic discovery.

### Why Pi Zero 2W for the camera?
The Pi Zero 2W connects via USB gadget ethernet (`192.168.7.2`), creating an isolated camera subnet. It runs a simple libcamera HTTP server — no ROS on the camera node, minimal attack surface, easy to swap out.

### Why PID depth hold on the onboard computer?
Latency matters for control loops. Keeping depth hold on the Pi (close to the FC) avoids round-trip delay through the topside.

---

## Network Layout

```
[DualShock 4] --USB--> [Laptop :ahmad@192.168.1.69]
                              |
                    ROS2/DDS (unicast)
                              |
                       [Pi 5 :hydromeda@192.168.1.70]
                              |
                     USB serial /dev/ttyACM0
                              |
                       [Pixhawk1 / ArduSub]
                              |
                         [ESCs / Thrusters]

[Pi Zero 2W] --USB gadget eth 192.168.7.2--> [Pi 5] --/photogrammetry/*--> [Laptop saver]
```

---

## Onboarding Checklist

- [ ] Clone both repos onto respective machines (see repo table above)
- [ ] Verify `hydromeda@192.168.1.70` and `ahmad@192.168.1.69` are reachable
- [ ] Verify DualShock 4 appears under `/dev/input/` on laptop
- [ ] Verify `/dev/ttyACM0` exists on Pi (Pixhawk connected)
- [ ] Verify Pi Zero 2W camera reachable: `curl http://192.168.7.2:8080/stream` from Pi
- [ ] Build Docker images on respective machines
- [ ] Run `./start-docker.sh` on both, open `http://192.168.1.69:8080` in browser
- [ ] Arm ROV (L1 + R1 on DS4), verify thrusters respond

---

## Known Issues / TODOs

- The web dashboard (`rov_dashboard`) shows telemetry but UI is minimal — no graphing, no depth plot
- Depth hold PID gains are hand-tuned; relay auto-tuner is implemented but not been run in a full pool test
- No automated tests — the system should be tested in a tank before any open-water dive
- Pi Zero 2W HTTP stream has no auth; treat the camera subnet as trusted-only
