#!/usr/bin/env bash

#activate ros environment
source /opt/ros/jazzy/setup.bash

# go to project folder
cd ~/Projects/Crazyflie/

# activate venv
source .venv/bin/activate

#activate ros venv
source ~/Projects/Crazyflie/ros2_ws/install/setup.bash

#Prompt for the waypoints file
cd ~/Projects/Crazyflie/ros2_ws/src/crazyflie-ros-navigation/crazyflie_goal_follower/maps/
codex exec --skip-git-repo-check - < prompt.txt

export GZ_SIM_RESOURCE_PATH="/home/argonaut/Projects/Crazyflie/simulation_ws/crazyflie-simulation/simulator_files/gazebo/"

# Overlay waypoints on the map before launching
python3 ../scripts/overlay_waypoints_on_map.py \
  --map-yaml greenhouse_map.yaml \
  --waypoints waypoints.txt \
  --output greenhouse_map_waypoints_small_points.ppm

xdg-open greenhouse_map_waypoints_small_points.ppm >/dev/null 2>&1 &

ros2 launch crazyflie_goal_follower goal_follower.launch.xml \
  enable_waypoint_publisher:=true \
  waypoints:=/home/argonaut/Projects/Crazyflie/ros2_ws/src/crazyflie-ros-navigation/crazyflie_goal_follower/maps/waypoints.txt
