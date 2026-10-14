## Robot Package Template

This is a GitHub template. You can make your own copy by clicking the green "Use this template" button.

It is recommended that you keep the repo/package name the same, but if you do change it, ensure you do a "Find all" using your IDE (or the built-in GitHub IDE by hitting the `.` key) and rename all instances of `my_bot` to whatever your project's name is.

Note that each directory currently has at least one file in it to ensure that git tracks the files (and, consequently, that a fresh clone has direcctories present for CMake to find). These example files can be removed if required (and the directories can be removed if `CMakeLists.txt` is adjusted accordingly).

## Driving the robot with the keyboard (ROS 2 Jazzy + Gazebo Harmonic)

The `diff_drive_controller` listens on `/diff_cont/cmd_vel` and expects a **TwistStamped**. The launch file also
starts `cmd_vel_relay.py`, which forwards a plain `/cmd_vel` (Twist) to it, so either of these works:

```bash
# Terminal 1: simulation (also opens RViz with the right config)
colcon build --symlink-install && source install/setup.bash
ros2 launch mobile_robot launch_sim.launch.py

# Terminal 2: keyboard (click into this terminal first so it has focus!)
source ~/dev_ws/install/setup.bash
ros2 run mobile_robot teleop.sh
```

Keys: `i` forward, `,` backward, `j`/`l` turn, `u`/`o` forward + turn, **`k` stop**, `q`/`z` faster/slower.
`teleop_twist_keyboard` sends one message per key press; `cmd_vel_timeout: 10.0` in `config/my_controllers.yaml`
keeps the robot moving after a press until you press `k` (simulation only).

Robot does not move? Run `ros2 run mobile_robot diagnose.sh --move` and read the `[FAIL]` lines. Also kill leftovers
of an earlier run before relaunching: `pkill -f "gz sim"; pkill -f ruby`.

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


## Moving the view (mouse)

- **Gazebo:** the window uses Gazebo's standard layout (`config/gui.config`) plus the Camera preview.
  Left-drag = pan, middle-drag (or Shift + left-drag) = orbit, scroll = zoom. Click a model to select it.
- **RViz:** use `rviz/camera.rviz` (toolbar with Interact / Move Camera). Left-drag = orbit, middle-drag = pan, scroll = zoom.
  Drag the panel borders to enlarge the Camera image / Depth image panels.

## Point cloud looks rotated (floor standing up like a wall)?

Gazebo's `rgbd_camera` point cloud uses the Gazebo axes (x forward, y left, z up), so the sensor in
`description/camera.xacro` must use `<gz_frame_id>camera_link</gz_frame_id>`, **not** `camera_link_optical`.


## Street world with people, lidar and a simple detector

`ros2 launch mobile_robot launch_sim.launch.py` now starts `worlds/street_world.sdf`: sidewalk, grass, road,
a building, trees, a mailbox and a box, a **walking pedestrian** (crosses 3 m in front of the robot, back and forth)
and a **standing woman** (Fuel model "Casual female"). The people are downloaded from Gazebo Fuel the first time
(internet needed, can take a minute; cached in `~/.gz/fuel`).

Launch arguments:

```bash
ros2 launch mobile_robot launch_sim.launch.py world:=camera_world.sdf   # the older, simple world
ros2 launch mobile_robot launch_sim.launch.py detector:=false           # no detector node
ros2 launch mobile_robot launch_sim.launch.py webcam:=true              # also publish a real webcam
```

What you see:

- **Gazebo:** Camera preview panel, plus the **Visualize lidar** panel that draws the blue lidar fan.
- **RViz** (`rviz2 -d $(ros2 pkg prefix mobile_robot)/share/mobile_robot/rviz/camera.rviz`): coloured point cloud
  of the scene and the people, the blue lidar scan (`/scan`), the colour and depth images, and red spheres with an
  "object x.x m" label (`/detections`).
- `scripts/object_detector.py` is a *very simple* lidar detector: it groups beams into objects closer than 4 m in
  the front +/-70 degrees. It does **not** recognise people, it just reports "something is here". Tune with
  `ros2 param set /object_detector detect_range 6.0`.
- Drive towards the people with the keyboard teleop (see above) and watch the point cloud and markers in RViz.

Notes: actors have no collision (the robot can drive through them) but the camera and lidar do see them.
The world has a road with crosswalk, three buildings, trees, lamps, benches, cars, a man walking along the
crosswalk and a woman (Fuel `Luca/FemaleVisitorWalk`) walking up and down the sidewalk, plus a standing woman.
If a walker floats or sinks, change the `z` values (1.0) of her waypoints in `worlds/street_world.sdf`.
RViz shows no lidar? Start it with `-d .../camera.rviz` (the launch file does this), and in Gazebo click the
refresh arrow in the "Visualize lidar" panel and pick `/scan`.
