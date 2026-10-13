## Robot Package Template

This is a GitHub template. You can make your own copy by clicking the green "Use this template" button.

It is recommended that you keep the repo/package name the same, but if you do change it, ensure you do a "Find all" using your IDE (or the built-in GitHub IDE by hitting the `.` key) and rename all instances of `my_bot` to whatever your project's name is.

Note that each directory currently has at least one file in it to ensure that git tracks the files (and, consequently, that a fresh clone has direcctories present for CMake to find). These example files can be removed if required (and the directories can be removed if `CMakeLists.txt` is adjusted accordingly).

## Driving the robot with the keyboard (ROS 2 Jazzy + Gazebo Harmonic)

The `diff_drive_controller` listens on `/diff_cont/cmd_vel` and expects a **TwistStamped**
message. `teleop_twist_keyboard` publishes a plain `Twist` on `/cmd_vel` by default, which
nothing is listening to, so the robot stays still. Run it like this:

```bash
# Terminal 1
colcon build --symlink-install && source install/setup.bash
ros2 launch mobile_robot launch_sim.launch.py

# Terminal 2 (wait until both controllers are "active": ros2 control list_controllers)
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p stamped:=true -p use_sim_time:=true \
  -r cmd_vel:=/diff_cont/cmd_vel
```

Then press `i` (forward), `u` (forward-left), `o` (forward-right), `k` (stop).
`use_sim_time:=true` matters: the stamps must use Gazebo's clock or the controller drops the commands as stale.
