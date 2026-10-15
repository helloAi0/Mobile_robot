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
ros2 run mobile_robot teleop_keys.py     # recommended: keeps sending, shows if the controller listens
# alternative: ros2 run mobile_robot teleop.sh   (stock teleop_twist_keyboard)
```

Keys (`teleop_keys.py`; the status line shows `controller listening` or `NO CONTROLLER`): `i` forward, `,` backward, `j`/`l` turn, `u`/`o` forward + turn, **`k` stop**, `q`/`z` faster/slower.
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


## Street world, physics, lidar and detector

`ros2 launch mobile_robot launch_sim.launch.py` starts `worlds/street_world.sdf` + Gazebo + RViz.

**World:** sidewalk, grass, road with crosswalk and cars, three buildings, trees, lamps, benches, kiosk, bus stop,
fences, a ramp and speed bumps. **Free bodies with real physics** (push them, knock them over): traffic cones,
barrels, crates (a stack), balls, a cardboard box. **Walking people** (Gazebo actors): two men and three women
(Fuel `Luca/FemaleVisitorWalk`). Actors are animations: camera and lidar see them but they have no collision.
Fuel models are downloaded on the first start (internet needed, cached in `~/.gz/fuel`).

Walkers floating or sinking? `ros2 run mobile_robot tune_actor_height.py 0.8` (women; smaller = lower, try
0.7 ... 1.1) or `... 1.0 man` for the men, then restart.

```bash
ros2 launch mobile_robot launch_sim.launch.py world:=camera_world.sdf   # the older, simple world
ros2 launch mobile_robot launch_sim.launch.py detector:=false rviz:=false webcam:=true
```

**How to read RViz** (`rviz/camera.rviz`, opened by the launch file):

- Grid = 1 m squares, so you can read distances.
- **Green dots = the LIDAR**: one ring around the robot. Each dot is a laser beam that hit something (720 beams,
  360 degrees, up to 12 m). No dots in a direction = nothing within 12 m there.
- **Camera 3D (floor removed)**: the depth camera as a coloured 3D cloud, only things standing on the floor
  (`scripts/cloud_filter.py` removes the floor from `/camera/points`).
- **Orange balls + "x.x m"** = `scripts/object_detector.py`: groups lidar dots into objects closer than 4 m in the
  front +/-70 degrees. It does not recognise people, only "something is here".
- Left panels: colour image and depth image of the camera.

Gazebo: click the refresh arrow in the "Visualize lidar" / "Camera preview" panels and pick `/scan` / `/camera/image`.

