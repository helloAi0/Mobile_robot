# Navigation: Nav2 tuned for this robot, and a patrol mission

```bash
ros2 launch mobile_robot_bringup launch_sim.launch.py nav2:=true patrol:=true     # patrols all rooms, forever
ros2 launch mobile_robot_bringup launch_sim.launch.py nav2:=true                  # click "2D Goal Pose" in RViz
ros2 run mobile_robot_bringup patrol.py --ros-args -p loops:=2                    # 2 laps (Nav2 already running)
```

`config/nav2_params.yaml` replaces the nav2_bringup defaults. Every number has a reason, written at the top of the file and checked by `test/test_nav2_config.py` (13 tests):

| Setting | Value | Reason |
|---|---|---|
| footprint | quadrilateral, nose 0.26 m ahead of the axle, 0.30 m wide at the front, 0.39 m at the axle | read from the URDF; the same polygon is used by the benchmark and checked equal |
| inflation radius / cost scaling | 0.55 m / 3.0 | larger than the circumscribed radius (0.30 m); steep falloff keeps the 1.2 m doorways passable (a test computes the cost at the doorway centre) |
| cruise speed | 0.35 m/s, 1.2 rad/s | the mux slows linearly inside 1 m anyway; planning below its limits (0.6 m/s) avoids fighting the guard. A test checks the stopping distance from cruise fits in the guard's slow band |
| acceleration | 1.0 m/s², decel 1.5 | at most the mux limit (1.2) and the controller limit (1.8): the smoother never asks for more than the wheels get |
| reverse | -0.2 m/s | the lidar is blind behind (pillars masked): back up slowly |
| planner | NavFn A*, `allow_unknown`, tolerance 0.3 m | the SLAM map is still growing while navigating |
| controller | DWB, 20 Hz, 1.8 s horizon | differential drive, no sideways speed |
| goal tolerance | 0.15 m, 0.3 rad | looser than odometry noise, much tighter than a doorway |
| obstacle sensing | `/scan`, mark to 5 m, clear to 6 m | the lidar sees 12 m, the costmap does not need all of it |

Command chain: `controller_server -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel_smoothed -> cmd_vel_mux -> wheels`. The mux keeps its safety layer over Nav2.

## Patrol (`scripts/patrol.py`, `config/patrol_house.yaml`)
Nine waypoints through the corridor and all five rooms. A goal that fails is retried twice, then skipped, and the patrol continues: one blocked door does not end the mission.
`test/test_patrol.py` checks against the geometry of the world file that every waypoint is free, at least 0.30 m + margin from obstacles, reachable,
off the wet tile, that all five rooms are visited, and that every leg is plannable and not absurdly long (one lap is about 60 m).
The retry/skip logic is plain Python and unit-tested.

**What is verified and what is not:** parameter consistency and route geometry are tested here. Whether the robot actually drives the route in Gazebo
(controller tuning, behaviour in doorways, recoveries) needs the simulator and has not been run.
