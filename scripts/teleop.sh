#!/usr/bin/env bash
# Keyboard control for the simulated robot. Run in its own terminal (the simulation must be running):
#   ros2 run mobile_robot teleop.sh
# Keys:  i = forward   , = backward   j = turn left   l = turn right   u / o = forward + turn   k = STOP
#        q / z = faster / slower
# Click into this terminal first, otherwise the key presses go to another window.
exec ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p stamped:=true -p use_sim_time:=true -r cmd_vel:=/diff_cont/cmd_vel "$@"
