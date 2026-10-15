## Robot Package Template

This is a GitHub template. You can make your own copy by clicking the green "Use this template" button.

It is recommended that you keep the repo/package name the same, but if you do change it, ensure you do a "Find all" using your IDE (or the built-in GitHub IDE by hitting the `.` key) and rename all instances of `my_bot` to whatever your project's name is.

Note that each directory currently has at least one file in it to ensure that git tracks the files (and, consequently, that a fresh clone has direcctories present for CMake to find). These example files can be removed if required (and the directories can be removed if `CMakeLists.txt` is adjusted accordingly).

## Quick start (ROS 2 Jazzy + Gazebo Harmonic)

```bash
# once: dependencies
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-teleop-twist-keyboard
# build (every new terminal needs:  source /opt/ros/jazzy/setup.bash && source ~/dev_ws/install/setup.bash)
cd ~/dev_ws && colcon build --symlink-install && source install/setup.bash

ros2 launch mobile_robot launch_sim.launch.py            # Gazebo + RViz + live map (SLAM); drive with the keyboard
ros2 launch mobile_robot launch_sim.launch.py auto:=true # the robot wanders by itself, avoids things and builds the map
ros2 launch mobile_robot launch_sim.launch.py nav2:=true # click "2D Goal Pose" in RViz: the robot drives there by itself
```

Other launch arguments: `world:=house_world.sdf|street_world.sdf|camera_world.sdf`, `gui:=false` (no Gazebo window:
drive and watch only from RViz), `rviz:=false`, `slam:=false`, `detector:=false`, `cloud:=false`, `webcam:=true`, `x:= y:=` (spawn point).
First start downloads the people models from Gazebo Fuel (internet needed once, cached in `~/.gz/fuel`).
Before relaunching, kill leftovers of an earlier run: `pkill -f "gz sim"; pkill -f ruby`.

## Three ways to drive (same in simulation and on hardware)

| Mode | How | What happens |
|---|---|---|
| **Manual** | second terminal: `ros2 run mobile_robot teleop_keys.py` | `i` forward, `,` back, `j`/`l` turn, `u`/`o` forward+turn, `k` stop, `q`/`z` speed, `a` auto on/off |
| **Autonomous** | `auto:=true` (or press `a` in the keyboard terminal) | drives forward, **stops when something is in front** (wall, furniture, a walking person), turns to the freer side, continues. SLAM remembers everything and draws the map |
| **Go to a point** | `nav2:=true`, then RViz toolbar **2D Goal Pose**, click + drag on the map | Nav2 plans a path on the SLAM map and drives there, avoiding obstacles |

`cmd_vel_mux.py` decides who drives: keyboard > Nav2 > autonomous. Pressing a key always wins; one second after `k` the
others take over again. Teleop window: click into it first so it has the focus; status line says `mux listening`.

**Map:** it grows in RViz (`MAP (built by SLAM)`). Save it: `ros2 run mobile_robot save_map.sh my_house` (-> `~/maps/my_house.yaml`).

## On a real robot

Everything above only uses standard topics, so the same nodes run on hardware:
`/scan` (LaserScan, frame `laser_frame`), `/odom` + TF `odom -> base_link`, and the robot drives from `/diff_cont/cmd_vel` (TwistStamped; or set the
mux parameters `output_topic:=/cmd_vel stamped:=false` for a plain Twist driver). Start your hardware driver, then
`ros2 launch mobile_robot autonomy.launch.py use_sim_time:=false auto:=true` (SLAM + mux + auto) and RViz with `rviz/camera.rviz`.
Nav2 runs with the default parameters of nav2_bringup (robot radius 0.22 m); copy and edit its params file if your robot is bigger.

## The worlds

- **`house_world.sdf` (default, for SLAM / Nav2):** a corridor with five rooms, 1.2 m doors, furniture, pillars, a ramp, free bodies with real
  physics (boxes, crates, barrels, cones, balls: push them) and four walking people. Walls give the lidar and SLAM clear features, rooms form loops for loop closure.
- **`street_world.sdf`:** open sidewalk and road, buildings, cars, trees, cones, ramp, five walking people.
- **`camera_world.sdf`:** tiny test world with three coloured shapes.

People are Gazebo "actors" (animated meshes). The camera and lidar see them, but they have **no collision**: the robot does not push them,
it *stops* because the lidar sees them.

**Walkers floating or sunk into the floor?** The height of the Fuel female mesh could not be verified without a running Gazebo, so it is tunable:
```bash
ros2 launch mobile_robot launch_sim.launch.py world:=actor_height_test.sdf slam:=false   # 7 women at different heights, lane n has n red cubes
python3 ~/dev_ws/src/mobile_robot/scripts/tune_actor_height.py 0.1          # use the z of the lane where the feet touch the floor
python3 ~/dev_ws/src/mobile_robot/scripts/tune_actor_height.py 1.0 man      # same for the men (default 1.0)
```

## Reading RViz (`rviz/camera.rviz`)

- **MAP (built by SLAM)**: black = walls, white = free floor, grey = unknown. **Magenta dots = the lidar** (720 beams, 360 degrees, up to 12 m); where
  they sit on a wall the beam hit it. Fixed frame is `odom`, the view follows the robot (`base_link`).
- **Camera 3D (floor removed)**: depth camera as a coloured 3D cloud (`scripts/cloud_filter.py` removes the floor). **Orange balls + "x.x m"**: objects
  the simple detector found within 4 m in front. Left panels: colour image, depth image. Green line = Nav2 path.
- Mouse: left-drag orbit, middle-drag pan, scroll zoom. In Gazebo click the refresh arrow of the "Visualize lidar" / "Camera preview" panels and pick `/scan` / `/camera/image`.

## Troubleshooting

- `ros2: command not found` / `Package 'mobile_robot' not found`: this terminal has no ROS loaded: `source /opt/ros/jazzy/setup.bash && source ~/dev_ws/install/setup.bash`.
- Robot does not move: start the keyboard terminal after the simulation says `diff_cont` is active; run `ros2 run mobile_robot diagnose.sh --move`.
- Point cloud looks rotated (floor like a wall): the camera sensor in `description/camera.xacro` must use `<gz_frame_id>camera_link</gz_frame_id>`.
- No map: `ros2 pkg prefix slam_toolbox` must work (see the apt line above).

## Optional: real webcam

If your computer has a camera: `sudo apt install ros-jazzy-v4l2-camera`, then `ros2 launch mobile_robot launch_sim.launch.py webcam:=true` (or
`ros2 launch mobile_robot webcam.launch.py video_device:=/dev/video0`) and tick **Webcam (optional)** in RViz (`/webcam/image_raw`).
