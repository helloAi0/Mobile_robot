#!/usr/bin/env bash
# Save the map slam_toolbox has built so far (needs ros-jazzy-nav2-map-server):
#   ros2 run mobile_robot save_map.sh            -> ~/maps/map.yaml + map.pgm
#   ros2 run mobile_robot save_map.sh my_house   -> ~/maps/my_house.yaml + my_house.pgm
mkdir -p "$HOME/maps"
exec ros2 run nav2_map_server map_saver_cli -f "$HOME/maps/${1:-map}" --ros-args -p save_map_timeout:=10.0
