# crazyflie-ros-navigation

This repository contains Crazyflie navigation-related packages and is based on the Bitcraze demo [Crazyflies' Adventures with ROS 2 and Gazebo](https://www.bitcraze.io/2024/09/crazyflies-adventures-with-ros-2-and-gazebo/), adapted from the original Humble/Harmonic setup to **Ubuntu 24.04 + ROS 2 Jazzy** as suggested by the Crazyswarm2 installation guidance.

Framework: ROS 2-based architecture using Crazyswarm2.

image

## Quick Start (First Flight Bring-up)

Use this minimal sequence after dependencies are installed, `crazyflies.yaml` is updated with the correct `uri`, and Lighthouse/radio prerequisites are complete.

```bash
cd /home/argonaut/Projects/Crazyflie/ros2_ws
colcon build --packages-select crazyflie_goal_follower
source install/setup.bash
ros2 launch crazyflie_goal_follower goal_follower_real.launch.xml odom_frame:=world
```

Optional keyboard teleop (separate terminal):

```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Repository Architecture

- ROS 2 nodes and launch files under `ros2_ws/src`
- Flight stack integration through `crazyswarm2`
- Simulation dependencies through `ros_gz_crazyflie` and `crazyflie-simulation`
- Firmware managed in `crazyflie-firmware` (kept outside the ROS 2 workspace)

## System Configuration

| Item | Configuration |
|---|---|
| Operating System | Ubuntu 24.04 LTS |
| ROS 2 Distribution | ROS 2 Jazzy |
| Version Note | Use the ROS 2 release supported by the current Crazyswarm2 repository |
| Flight Stack | Crazyswarm2 |

## Required Packages

- `gazebo`
- `crazyswarm2`
- `ros_gz_crazyflie`
- `crazyflie-simulation`
- `crazyflie-firmware` (clone outside ROS 2 workspace)

Additional requirements:

```bash
sudo apt-get install libboost-program-options-dev libusb-1.0-0-dev python3-colcon-common-extensions
sudo apt-get install ros-jazzy-motion-capture-tracking ros-jazzy-tf-transformations
sudo apt-get install ros-jazzy-teleop-twist-keyboard
pip3 install cflib transforms3d rowan nicegui==1.4.2
```

## Setup Workflow

1. Install the base platform
    - Install Ubuntu 24.04 LTS, ROS 2 Jazzy, Gazebo, and required core tools.
    - Follow the Crazyswarm2 installation guide for Crazyflie-related dependencies.
2. Pull the repositories
    - Clone all required packages listed above.
    - Keep `crazyflie-firmware` outside the ROS 2 workspace.
3. Build the workspace
    - Build `ros2_ws` following the procedures recommended by Crazyswarm2 and Bitcraze docs.

## Install

Follow the installation steps for the dependencies in:

- Crazyswarm2 installation guide: https://imrclab.github.io/crazyswarm2/installation.html#first-installation
- Bitcraze ROS 2 and Gazebo guide: https://www.bitcraze.io/2024/09/crazyflies-adventures-with-ros-2-and-gazebo/
- ROS 2 Jazzy installation guide: https://docs.ros.org/en/jazzy/Installation.html
- Gazebo Harmonic installation guide: https://gazebosim.org/docs/harmonic/install_ubuntu/

## Crazyflie Workspace Layout

```text
/Crazyflie Project Folder
├── crazyflie-firmware/
├── ros2_ws/
│   ├── src/
│   │   ├── crazyflie-ros-navigation/
│   │   │   └── crazyflie_ros2_test/
│   │   │   └── crazyflie_ros2_goal_follower/
│   │   ├── crazyflie_ros2_multiranger/
│   │   ├── crazyswarm2/
│   │   └── ros_gz_crazyflie/
│   ├── build/
│   ├── install/
│   └── log/
└── simulation_ws/
    └── crazyflie-simulation/
```

## Deployment Prerequisites

- Crazyflie firmware is up to date
- Crazyradio firmware is up to date
- Lighthouse base stations are configured before ROS bring-up
- Crazyradio Linux USB permissions are configured

## Deployment Workflow

1. Configure radio USB permissions
    - Follow: https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/installation/usb_permissions/
2. Update Crazyswarm2 robot configuration
    - Replace `crazyflies.yaml` in `crazyswarm2/crazyflie/config` with your deployment config.
    - Set the `uri: radio://...` field for your Crazyflie.
3. Discover the correct URI with CFclient
    - Plug in Crazyradio and power on the Crazyflie.
    - Open CFclient and press **Scan**.
    - In the dropdown next to **Connect**, copy the detected URI (example: `radio://0/80/2M/E7E7E7E7E7`).
    - Paste the exact URI into `crazyflies.yaml`.

## Try Keyboard Teleoperation

In terminal 1:

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch crazyflie_examples keyboard_velmux_launch.py
```

In terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Try Goal Follower Node

```bash
ros2 launch crazyflie_goal_follower goal_follower_real.launch.xml odom_frame:=world
```

## Goal Follower `go_to` Services

`goal_follower_node` sends Crazyflie high-level commands by calling:

- `/<robot_prefix>/takeoff`
- `/<robot_prefix>/go_to`
- `/<robot_prefix>/land`

With `goal_follower_real.launch.xml`, the default `robot_prefix` is `crazyflie_real`.
That launch includes Crazyswarm2's `crazyflie/launch/launch.py` with `backend:=cflib`, which starts the real `crazyflie_server.py`.
For the configured robot name, the server creates `/crazyflie_real/go_to` and implements it by forwarding the request to cflib's high-level commander:

```text
cf.high_level_commander.go_to(x, y, z, yaw, duration, relative, group_mask)
```

So on real hardware, `go_to` is handled by the Crazyflie server and sent to the firmware high-level commander. It does not use `/cmd_vel`.

`crazyflie_goal_follower` also includes `service_compat`, which exposes the same `takeoff`, `go_to`, and `land` service names by publishing `/cmd_vel`. Use it only as a fallback when those services are missing:

```bash
ros2 launch crazyflie_goal_follower goal_follower_real.launch.xml enable_service_compat:=true
```

Do not enable `service_compat` when Crazyswarm2 is already providing `/<robot_prefix>/go_to`, because two nodes cannot own the same service name.

## Run `test_node`

`test_node` in `crazyflie_ros2_test` expects these services:

- `/<robot_prefix>/takeoff`
- `/<robot_prefix>/go_to`
- `/<robot_prefix>/land`

Build and run from your ROS 2 workspace root:

```bash
cd /home/argonaut/Projects/Crazyflie/ros2_ws
colcon build --packages-select crazyflie_ros2_test
source install/setup.bash
ros2 run crazyflie_ros2_test test_node --ros-args -p robot_prefix:=crazyflie
```

Use a different namespace if needed:

```bash
ros2 run crazyflie_ros2_test test_node --ros-args -p robot_prefix:=cf1
```
