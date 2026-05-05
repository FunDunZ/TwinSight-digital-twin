# TwinSight Digital Twin

A real-time VR digital twin of the HiWonder TurboPi mecanum robot, built with ROS 2 Humble and Unity 6. Control the robot from a Meta Quest 3 headset and watch a 3D model mirror its movements live.

> **Note on Docker:** Docker is not currently set up on this development machine. ROS 2 runs directly on the robot's Raspberry Pi 5. Docker will become relevant in future development for running path planning algorithms (Nav2, SLAM) and offloading computationally heavy tasks away from the Pi — see [Why Docker?](#why-docker) below.

---

## Quick Installation

### Prerequisites

Install these on your Windows PC before starting:

- [Git](https://git-scm.com/)
- [Unity Hub](https://unity.com/download) + Unity 6 (with **Android Build Support** and **OpenXR** modules)
- [Meta Quest Developer Hub](https://developer.oculus.com/documentation/unity/unity-env-device-setup/) (for deploying to Quest 3)

> Docker is not required for the current setup — skip it for now. See [Why Docker?](#why-docker) if you want to set it up for future development.

### 1. Clone the repo

```bash
git clone https://github.com/FunDunZ/TwinSight-digital-twin.git
cd TwinSight-digital-twin
```

### 2. Build the ROS 2 workspace (on the robot's Raspberry Pi)

SSH into the TurboPi and build the workspace:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### 3. Run the robot

```bash
# Terminal 1 — full robot bringup
ros2 launch bringup bringup.launch.py

# Terminal 2 — Unity bridge
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=<robot-ip>
```

### 4. Open the Unity project

1. Open **Unity Hub → Add project from disk** → select `TwinSight_dev_v1_unity/`
2. Go to **Robotics → ROS Settings** and set the ROS IP to your robot's LAN IP, port `10000`
3. Open the scene `Assets/Scenes/HighFaceCount`
4. Press **Play** to test in the Editor, or build to your Quest 3

---

## Documentation

Having trouble? The full guides are here:

| Document | What's In It |
|---|---|
| [documentation/twinsight-digital-twin.md](documentation/twinsight-digital-twin.md) | **Main project docs** — hardware specs, ROS 2 guide, Unity guide, full installation walkthrough, current limitations, achievements, future plans |
| [TurboPi_Documentation.md](TurboPi_Documentation.md) | **Robot-specific docs** — TurboPi hardware, HiWonder SDK details, sensor wiring, factory software |
| [documentation/system-architecture.md](documentation/system-architecture.md) | System architecture diagrams |

> If you are new to ROS 2 or Unity — start with [documentation/twinsight-digital-twin.md](documentation/twinsight-digital-twin.md). It covers everything from scratch for CS students with no prior robotics experience.

---

## Project Structure

```
TwinSight-digital-twin/
├── documentation/             # Project documentation
├── turbopi_ros2/ros2_ws/src/  # ROS 2 packages
└── TwinSight_dev_v1_unity/    # Unity project
```

---

## Why Docker?

Docker is not needed for the current workflow because all ROS 2 nodes run directly on the robot's Raspberry Pi 5. However, Docker will be valuable in the next phases of development:

| Future Use Case | Why Docker Helps |
|---|---|
| **Path planning (Nav2 / SLAM)** | Nav2 and `slam_toolbox` are computationally heavy — running them on a PC in Docker offloads the Pi and allows much faster iteration |
| **Offloading vision/AI nodes** | YOLOv11 and LLM pipeline nodes stress the Pi's CPU/RAM — a Docker container on a PC with a GPU can run these instead |
| **Consistent development environment** | Everyone on the team gets identical ROS 2 Humble + dependencies without manual installs |
| **Multi-machine ROS 2 network** | Docker with `network_mode: host` means the container's ROS nodes appear on the same LAN as the robot, enabling seamless topic sharing |

To set up Docker when the time comes, see the full instructions in [documentation/twinsight-digital-twin.md → Section 8.2](documentation/twinsight-digital-twin.md) and install [Docker Desktop](https://www.docker.com/products/docker-desktop/).

---

## Contributors
Vinh Le (Project Lead & Founder)
Cardin Tran (IMU Integration)
