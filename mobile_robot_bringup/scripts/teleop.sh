#!/usr/bin/env bash
# Stock keyboard teleop (teleop_twist_keyboard) wired to the robot's command mux. Prefer: ros2 run mobile_robot_bringup teleop_keys.py
#   i forward  , backward  j/l turn  u/o forward+turn  k stop   (one message per key press: hold the key to keep moving)
exec ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_teleop "$@"
