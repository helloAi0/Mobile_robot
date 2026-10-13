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


## Camera preview (Gazebo + RViz)

`launch_sim.launch.py` now starts Gazebo with `worlds/camera_world.sdf`. Gazebo's stock `empty.sdf`
does not load the Sensors system, so the camera never rendered. The new world loads it, adds a few
objects in front of the robot and opens a floating **Camera preview** panel in the Gazebo window
(if it is empty, pick `/camera/image_raw` in its drop-down).

For ROS 2 / RViz the camera is an `rgbd_camera` (Gazebo Harmonic). It is bridged as
`/camera/image_raw`, `/camera/depth/image_raw`, `/camera/points` (coloured point cloud) and `/camera/camera_info`.
Do not use `libgazebo_ros_camera.so` or `<plugin ... gazebo_ros_...>` from older (Gazebo Classic) tutorials: Harmonic cannot load them.

```bash
rviz2 -d $(ros2 pkg prefix mobile_robot)/share/mobile_robot/rviz/camera.rviz   # image + depth + coloured 3D points
```

Manual checks:

```bash
ros2 topic hz /camera/image_raw          # should show ~10 Hz
rviz2                                    # Add -> By topic -> /camera/image_raw -> Image
                                         # (Fixed Frame: base_link)
```

Quick checks if there is no image: `gz topic -l | grep camera` (should list `/camera/image_raw`)
and `ros2 topic list | grep camera`. If Gazebo crashes or the image is black on a machine without
a GPU (VM / WSL), change `ogre2` to `ogre` in the Sensors plugin of `worlds/camera_world.sdf`.
