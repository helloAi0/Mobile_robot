# 10-minute demo script

Prepare (once, 5 minutes before): `pkill -f "gz sim"; pkill -f ruby`, start the simulation, wait until Gazebo shows the house and RViz shows the map growing. First start downloads the people models (internet), later starts are fast.

Terminals: **T1** simulation, **T2** keyboard. Keep Gazebo on the left, RViz on the right.

| Min | Do | Say |
|---|---|---|
| 0 | T1: `ros2 launch mobile_robot_bringup launch_sim.launch.py` | "One launch file starts the physics simulation, the sensors, the controller stack, SLAM and RViz. Same software I run on the real robot." |
| 1 | Point at Gazebo: corridor, five rooms, furniture, boxes and barrels | "Walls give SLAM real features. The boxes are rigid bodies with mass, the robot can push them. The people walk around and the lidar and camera see them." |
| 2 | T2: `ros2 run mobile_robot_bringup teleop_keys.py`, drive with `i` `j` `l` `m` | "Keyboard teleop with live status. The mux between all drivers and the wheels adds the safety layer." |
| 3 | Drive at the wall: status shows `slowing`, then `obstacle`; back up with `m` | "The obstacle guard slows and then blocks forward motion; backing out is always possible. Without a lidar signal the robot refuses to move." |
| 4 | RViz: point at the map, magenta lidar dots, orange detection balls with distances, the camera and depth images, the floor-free 3D cloud | "Black = wall, white = free, grey = unknown. The map is built live from lidar and wheel odometry." |
| 5 | Press `p` (autopilot) and let it go | "Now it explores by itself. When something is in front, a wall or a person, it stops, turns to the freer side and continues. If it ever gets stuck it backs out." |
| 6 | Let it run into a walking person's path | "People are actors without collision, so the robot does not push them: it stops because the lidar sees them." |
| 7 | `Ctrl-C` T2, stop T1; `ros2 launch mobile_robot_bringup launch_sim.launch.py nav2:=true`; RViz **2D Goal Pose** on the map | "Same robot with Nav2: I click a goal on the map, it plans and drives, avoiding obstacles." |
| 8 | Third terminal: `ros2 run mobile_robot_bringup estop.sh` then `... estop.sh release` | "Software emergency stop. On hardware there is also a physical stop." |
| 9 | Show `docs/HARDWARE.md`, `ros2 launch mobile_robot_bringup hardware.launch.py` (mock) | "To go to hardware I provide a motor driver and a lidar driver. Everything else is unchanged. CI runs the tests and a mock-hardware integration test on every push." |

Acceptance run (what must work before you present):
1. `ros2 control list_controllers`: `diff_cont` and `joint_broad` active.
2. Keyboard: `i` forward, `m` straight back, `j`/`l` spin, `k` stop.
3. RViz map grows while driving; magenta dots follow the walls.
4. `p`: robot wanders for 2 minutes without getting stuck.
5. `nav2:=true`: a goal in the corridor is reached.
6. Women walk with their feet on the floor (else `tune_actor_height.py`).
If something fails run `ros2 run mobile_robot_bringup diagnose.sh --move` and send the output.

---

## Before you record (pre-flight, 3 minutes)

```bash
# terminal 1: the simulation (wait until RViz and Gazebo are up and the log stops scrolling, about 30 s)
source /opt/ros/jazzy/setup.bash && source ~/dev_ws/install/setup.bash
ros2 launch mobile_robot_bringup launch_sim.launch.py

# terminal 2: automatic check, no keyboard needed. The robot drives about 1 m. Every line must say PASS.
source /opt/ros/jazzy/setup.bash && source ~/dev_ws/install/setup.bash
ros2 run mobile_robot_bringup selftest.py

# terminal 3: drive it yourself. Type INSIDE this terminal, not in Gazebo or RViz.
source /opt/ros/jazzy/setup.bash && source ~/dev_ws/install/setup.bash
ros2 run mobile_robot_bringup teleop_keys.py
```

Suggested 90-second recording: (1) Gazebo + RViz side by side, (2) drive the corridor with `w` `a` `d` `m`, (3) push a box, cross the ramp,
(4) stop in front of a wall: the safety layer slows and stops it, (5) press `p`: autopilot explores and the SLAM map grows in RViz,
(6) run `ros2 run mobile_robot_bringup metrics_logger.py` in parallel and show the summary at the end.

## Warnings in the launch log that are harmless

| message | meaning |
|---|---|
| `gz_frame_id ... not defined in SDF. Copying` | Gazebo copies a custom tag; sensors work |
| `IMU sensor 'imu' not found in hardware_info, skipping` | the IMU is published by the bridge, not by ros2_control |
| `Desired controller update period ... slower than the gazebo simulation period` | normal: physics runs at 500 Hz, the controller at 50 Hz |
| `Can't accept new commands. subscriber is inactive` / `TwistStamped with zero timestamp` | printed once while the controller starts |
| `waiting for service /controller_manager/list_controllers` | spawners waiting for Gazebo to load the robot |

## Problems seen on a real run, and the fix

| symptom | cause | fix |
|---|---|---|
| no lidar and no point cloud in RViz, log says `incompatible QoS ... RELIABILITY` | publishers were best-effort, RViz asks for reliable | fixed: `/scan` and `/camera/points_filtered` are published reliable |
| RViz error `active samplers with a different type refer to the same texture image unit`, SLAM map invisible | graphics driver (typical on WSL2) cannot compile RViz's map shader | the launch now uses software OpenGL for RViz on WSL. Force it: `export MOBILE_ROBOT_RVIZ_GL=software` |
| robot does not move, teleop line stays `STOPPED`, mux says `idle` | keys were typed in another window: teleop reads the keyboard only in its own terminal | click the teleop terminal first; run `selftest.py` to prove the robot itself works |
| simulation is slow (real-time factor below 1) | software rendering of cameras and lidar | `gui:=false` to hide the Gazebo window; close other apps; `world:=camera_world.sdf` for a light world |
