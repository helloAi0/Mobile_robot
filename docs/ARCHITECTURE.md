# Architecture

```
                    SIMULATION (launch_sim.launch.py)                         REAL ROBOT (hardware.launch.py)
              ┌──────────────────────────────────────┐                  ┌───────────────────────────────────┐
              │ Gazebo Harmonic: world, physics,     │                  │ your motor driver  (ros2_control)  │
              │ rgbd camera, gpu lidar, people       │                  │ your lidar driver  -> /scan        │
              │ gz_ros2_control (wheels)             │                  │ (or MOCK: simulated wheels)        │
              └───────┬──────────────────────────────┘                  └──────────────┬────────────────────┘
        ros_gz_bridge │ /scan /camera/* /clock                                          │ /scan
                      ▼                                                                 ▼
   ┌──────────────────────────── identical on both sides ─────────────────────────────────────────────────┐
   │ ros2_control: diff_drive_controller (diff_cont) + joint_state_broadcaster -> /diff_cont/odom, TF odom->base_link
   │ robot_state_publisher (URDF/xacro, hardware:=sim|mock|real)                                               │
   │                                                                                                           │
   │  keyboard  /cmd_vel_teleop ─┐                                                                             │
   │  Nav2      /cmd_vel*  ──────┼─► cmd_vel_mux.py ─► safety ─► /diff_cont/cmd_vel (TwistStamped)             │
   │  autopilot /cmd_vel_auto ───┘   priority         e-stop, obstacle guard (front+rear), limits, watchdog   │
   │                                                                                                           │
   │  slam_toolbox  /scan + odom -> /map, TF map->odom          Nav2 (planner, controller, costmaps)           │
   │  auto_explore.py  /scan + /odom -> wander, stop for people, back out when stuck                           │
   │  object_detector.py -> /detections     cloud_filter.py -> /camera/points_filtered     RViz               │
   └───────────────────────────────────────────────────────────────────────────────────────────────────────────┘
```

**Design rule:** everything above the dashed line (simulator or hardware) is the only part that differs.
The controller configuration (`mobile_robot_bringup/config/my_controllers.yaml`), the mux, SLAM, Nav2, the autopilot and RViz are the same files in simulation and on the robot,
so what is demonstrated in Gazebo is the software that runs on the machine.

## Nodes
| Node | File | Job |
|---|---|---|
| `cmd_vel_mux` | `mobile_robot_bringup/scripts/cmd_vel_mux.py` | single place that talks to the wheels: priority keyboard > Nav2 > autopilot; e-stop; obstacle guard; fail-safe without lidar; speed and acceleration limits; copies `/diff_cont/odom` to `/odom` |
| `auto_explore` | `mobile_robot_bringup/scripts/auto_explore.py` | reactive wandering: forward, STOP at obstacles/people, turn to the freer side, back out when stuck (no progress in 4 s) |
| `teleop_keys` | `mobile_robot_bringup/scripts/teleop_keys.py` | keyboard driver with live status |
| `object_detector` | `mobile_robot_bringup/scripts/object_detector.py` | groups lidar beams into objects in front: orange markers with distance |
| `cloud_filter` | `mobile_robot_bringup/scripts/cloud_filter.py` | removes the floor from the depth-camera point cloud |
| slam_toolbox | `mobile_robot_bringup/config/slam_toolbox.yaml` | online mapping, loop closure |
| Nav2 | nav2_bringup defaults | goal navigation on the SLAM map |

## Topics
| Topic | Type | Meaning |
|---|---|---|
| `/scan` | LaserScan | lidar, frame `laser_frame`, 720 beams, 0.15–12 m |
| `/diff_cont/cmd_vel` | TwistStamped | the ONLY input of the wheels (written by the mux) |
| `/cmd_vel_teleop`, `/cmd_vel` (Nav2), `/cmd_vel_auto` | Twist | driver inputs of the mux |
| `/diff_cont/odom`, `/odom` | Odometry | wheel odometry (from real wheel feedback) |
| `/emergency_stop` | Bool | `true` = stop and hold, `false` = release (latched) |
| `/mux_status` | String | `<driver> <limit>` e.g. `teleop slowing` |
| `/auto_mode` | Bool | autopilot on/off |
| `/map` | OccupancyGrid | SLAM map |
| `/detections` | MarkerArray | orange markers |
| `/camera/image_raw`, `/camera/depth/image_raw`, `/camera/points`, `/camera/points_filtered` | Image / PointCloud2 | camera |

## Safety layers (cmd_vel_mux.py)
1. `/emergency_stop` holds zero speed until released.
2. Forward motion is slowed linearly from 1.0 m to 0.38 m in front (±28° cone) and blocked closer; reversing is blocked closer than 0.30 m behind; turning on the spot is always allowed, so you can always get out.
3. No fresh `/scan` (older than 1 s) = no motion (parameter `require_scan`).
4. Speed limits 0.6 m/s / 2.0 rad/s and acceleration limits; braking is immediate.
5. Commands older than 0.5 s are ignored by the mux and by the controller (`cmd_vel_timeout`): if a driver crashes the robot stops.

All limits are ROS parameters of `cmd_vel_mux` (`ros2 param list /cmd_vel_mux`).
