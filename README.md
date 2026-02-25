# crazyflie-ros-navigation

This repository contains Crazyflie navigation-related packages.

## Configuration

- Ubuntu 24.04 LTS
- ROS 2 Jazzy
- Crazyswarm2

## Required Packages

- `gazebo`
- `crazyswarm2`
- `ros_gz_crazyflie`
- `crazyflie-simulation`
- `crazyflie-firmware` (cloned outside ROS 2 workspace)

## Install

Follow the installation steps for the dependencies in:

- https://www.bitcraze.io/2024/09/crazyflies-adventures-with-ros-2-and-gazebo/
- https://imrclab.github.io/crazyswarm2/installation.html#first-installation
- https://docs.ros.org/en/jazzy/Installation.html
- https://gazebosim.org/docs/harmonic/install_ubuntu/

## Crazyflie Workspace Layout

```text
/Crazyflie Project Folder
├── crazyflie-firmware/
├── ros2_ws/
│   ├── src/
│   │   ├── crazyflie-ros-navigation/
│   │   │   └── crazyflie_ros2_test/
│   │   ├── crazyflie_ros2_multiranger/
│   │   ├── crazyswarm2/
│   │   └── ros_gz_crazyflie/
│   ├── build/
│   ├── install/
│   └── log/
└── simulation_ws/
    └── crazyflie-simulation/
```

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

## TODO

1. Start with 2D navigation.
Treat the greenhouse as a 2D navigation problem by using a fixed `z`.

Add:
- A static occupancy map of the greenhouse layout (walls, racks, hard obstacles).
- Emergency stop / hover.
- Slow down when obstacles are near.
- Reject `go_to` commands when the direct path is blocked.
