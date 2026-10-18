# mobile_robot: simulation to hardware (ROS 2 Jazzy, Gazebo Harmonic)

A differential-drive robot with lidar and depth camera, **the same software in simulation and on the real robot**:
live SLAM mapping, Nav2 goal navigation, an autonomous obstacle-avoiding mode that stops for walking people, a safety layer
(e-stop, obstacle guard, fail-safe on sensor loss), a mock-hardware mode and a hardware bring-up path.

| | |
|---|---|
| **Simulation** | house world with five rooms, hinged doors, friction patches, a ramp, textured props (pallet, boxes, bins, barrels, cones), free-body physics and walking people; street world. Sprung drive wheels, swivel caster, noisy lidar / camera / IMU |
| **Mapping** | `slam_toolbox` live map in RViz, loop closure, `save_map.sh` |
| **Navigation** | Nav2: click **2D Goal Pose**, the robot plans and drives |
| **Autonomy** | `auto:=true`: drives, stops for obstacles and people, turns, continues, backs out when stuck |
| **Safety** | `cmd_vel_mux`: e-stop, front/rear obstacle guard, no-lidar = no motion, speed + acceleration limits, command watchdogs |
| **Perception views** | magenta lidar over the map, floor-free coloured 3D cloud, colour + depth image, object markers with distance |
| **Hardware path** | `hardware.launch.py` (mock or real motor driver), systemd + udev, Docker, CI with tests |
| **Quality** | 116 tests: safety logic, autopilot, filters, robot description in sim/mock/real, an independent physics check, SDF schema lint; CI; integration test on mock hardware |

## Quick start
```bash
bash mobile_robot_bringup/scripts/install_deps.sh                                      # apt packages (once)
source /opt/ros/jazzy/setup.bash          # ALWAYS first: without it you get "ament_cmake not found" / "ros2: command not found"
bash ~/dev_ws/src/mobile_robot/build.sh && source ~/dev_ws/install/setup.bash

ros2 launch mobile_robot_bringup launch_sim.launch.py                     # terminal 1: Gazebo + RViz + live map
ros2 run mobile_robot_bringup selftest.py                                 # terminal 2: automatic drive test, every line must say PASS
ros2 run mobile_robot_bringup teleop_keys.py                              # terminal 2: keyboard. Type IN THIS terminal. i/w fwd, m/s/, back, j/l turn, k stop, p autopilot
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix mobile_robot_bringup)/share/mobile_robot_bringup/rviz/camera.rviz \
     --ros-args -p use_sim_time:=true                                     # RViz alone (the launch file already starts it; use this if you closed it)
ros2 launch mobile_robot_bringup launch_sim.launch.py auto:=true         # robot explores by itself
ros2 launch mobile_robot_bringup launch_sim.launch.py nav2:=true         # RViz "2D Goal Pose"
ros2 run mobile_robot_bringup estop.sh                                    # emergency stop   (estop.sh release = go on)
```
Every command, with options: **[docs/COMMANDS.md](docs/COMMANDS.md)**. Presentation script: **[docs/DEMO.md](docs/DEMO.md)**.
How it fits together: **[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)**. Going to hardware: **[docs/HARDWARE.md](docs/HARDWARE.md)**.

Each new terminal needs `source /opt/ros/jazzy/setup.bash && source ~/dev_ws/install/setup.bash`. Before relaunching: `pkill -f "gz sim"; pkill -f ruby`.
The first start downloads the people models from Gazebo Fuel (internet once, cached in `~/.gz/fuel`).

## What I built (vs. what I reused)

Reused, standard stacks: ROS 2 Jazzy, Gazebo Harmonic, ros2_control, slam_toolbox, Nav2.
Written in this repo:
- `cmd_vel_mux.py`: priority command mux with a safety layer (e-stop, obstacle guard, no-scan fail-safe, speed/accel limits).
- `auto_explore.py`: reactive autopilot with stuck recovery.
- `cloud_filter.py`: floor-removing point cloud filter.
- One URDF for sim, mock and real hardware (ros2_control), with a mock bring-up that needs no robot.
- `metrics_logger.py`: measures a run (distance, explored area, min clearance, stops) and writes a CSV.
- Robot description with generated mass properties (full inertia tensors, parallel-axis theorem), sprung drive wheels, offset swivel caster, sensor noise models (`mobile_robot_description`).
- Worlds and their generators (`mobile_robot_gazebo`), an importer for textured props from osrf/gazebo_models, and the tooling below.
- 116 tests, CI, Docker, systemd/udev deployment files.

## How it is verified
| claim | evidence | where |
|---|---|---|
| the robot drives straight, turns the right way, climbs the 7 deg ramp, stops in 7 cm from full speed, suspension settles in 0.5 s | independent MuJoCo physics check, 18 measured checks, fails on a deliberately wrong wheel axis | `mobile_robot_description/tools/physics_check.py`, `make physics` |
| the suspension spring is real in Gazebo | read from the Gazebo source: sdformat converts it, the DART backend applies it | [docs/PHYSICS.md](docs/PHYSICS.md) |
| worlds and models are valid SDF | checked against the official sdformat schema; the linter is itself tested on planted errors | `mobile_robot_gazebo/tools/sdf_lint.py` |
| safety layer logic (e-stop, obstacle guard, fail-safe, limits) | unit tests | `test/test_mux_safety.py` |
| the whole running system | `selftest.py` drives the robot through the real command path and reports PASS/FAIL per subsystem | `docs/DEMO.md` |

Scope, stated plainly: validated in simulation and by unit tests; **not yet validated on physical hardware**. The physics check uses a different
engine from Gazebo, so it proves the robot description (axes, mass, spring, motors, geometry), not every contact detail of Gazebo.

## Measuring a run

```bash
ros2 run mobile_robot_bringup metrics_logger.py --ros-args -p csv:=$HOME/run1.csv   # Ctrl-C prints the summary
```

## Reading RViz (`mobile_robot_bringup/rviz/camera.rviz`)
* **MAP**: black = wall, white = free, grey = unknown. **Magenta dots = lidar** (720 beams, 360°, 0.15-12 m). Grid squares are 1 m.
* **Camera 3D (floor removed)**: the depth camera as a coloured cloud (`cloud_filter.py` drops the floor). **Orange balls + "x.x m"**: objects found in front. Green line: Nav2 path.
* Panels on the left: colour and depth image. Mouse: left-drag orbit, middle-drag pan, scroll zoom. `gui:=false` runs without the Gazebo window.
* Gazebo panels: click the refresh arrow of "Visualize lidar" / "Camera preview" and pick `/scan` / `/camera/image`.

## Worlds (`mobile_robot_gazebo/worlds/`, generated by `mobile_robot_gazebo/tools/`)
`house_world.sdf` (default; corridor + 5 rooms, 1.2 m doors, furniture, ramp, 4 walking people), `street_world.sdf` (outdoor),
`camera_world.sdf` (tiny test), `actor_height_test.sdf` (calibration). People are Gazebo actors: animated, seen by camera and lidar, **no collision**
(the robot stops for them because the lidar sees them). If a walker floats or sinks, see "Walkers floating" in docs/COMMANDS.md.

## Repository layout
```
mobile_robot_description/   urdf/ (xacro: chassis, suspension, caster, sensors, ros2_control), meshes/ (visual), tools/ (inertia + mesh generators)
mobile_robot_gazebo/        worlds/ (house, street, calibration), models/ (textured props from osrf/gazebo_models, CC BY 3.0; docs/ASSETS.md), launch/gazebo.launch.py, scripts/ (scan_degrader, tune_actor_height), tools/ (world generators), config/gui.config
mobile_robot_bringup/       launch/ (launch_sim, hardware, autonomy), config/ (controllers, slam), rviz/, scripts/ (cmd_vel_mux, auto_explore, teleop_keys, cloud_filter, estop, metrics, ...), test/ (pytest)
deploy/ (systemd, udev)  docs/  Dockerfile  Makefile  .github/workflows/ci.yml
```

## Troubleshooting
* `ros2: command not found` / `Package 'mobile_robot_bringup' not found`: this terminal has no ROS loaded (`source` lines above).
* The robot does not move: start the keyboard terminal when the launch log says `diff_cont` is active; the status line must show `mux OK`; run `ros2 run mobile_robot_bringup diagnose.sh --move`.
* Robot is stuck: back out with `m`; the status line shows `obstacle` when the guard blocks forward motion. In autopilot the robot backs out by itself.
* No map: `ros2 pkg prefix slam_toolbox` must work (`mobile_robot_bringup/scripts/install_deps.sh`).
* Point cloud looks rotated: the sensor in `mobile_robot_description/urdf/camera.xacro` must use `<gz_frame_id>camera_link</gz_frame_id>`.
* Optional webcam: `sudo apt install ros-jazzy-v4l2-camera`, then `webcam:=true` and tick "Webcam (optional)" in RViz.

License: Apache-2.0 (see LICENSE.md).
