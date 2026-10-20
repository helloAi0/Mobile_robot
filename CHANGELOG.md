# Changelog

## 1.1.1
Fixes from the first run on a real Jazzy / Gazebo Harmonic install:
- `ekf:=true` crashed: `ekf.yaml` mixed integers into lists of doubles, which the ROS parameter parser rejects. All lists are floats now; a test checks every config file.
- `nav2:=true` aborted: the installed Nav2 release starts `collision_monitor`, `route_server` and docking from `navigation_launch.py`, none of which were configured. Nav2's seven navigation nodes are now started explicitly.
- `/imu` had no data: the worlds lacked `gz-sim-imu-system` (selftest reported "IMU: NO DATA", the EKF would have been starved). Added to all worlds and generators.
- `selftest.py` makes room (reverses first) and reports a slowed drive as the safety layer, not as dead wheels.

## 1.1.0
- **Benchmark** (`tools/benchmark.py`): N autonomous runs, coverage / collisions / person contacts / minimum clearance / time, results table + figure in the README. 2D backend (real control code, geometry read from `house_world.sdf`) and Gazebo backend.
- **Autopilot**: yields to something that approaches by itself (person contacts per run 3.90 → 3.25) and makes room before turning on the spot (found by the benchmark: a wall-collision regression, fixed).
- **EKF** (`ekf:=true`): `robot_localization` fusing wheel velocities + IMU gyro, with covariances and frame ownership; heading error on the wet tile 12.6° → 1.0° in the model.
- **Nav2** tuned for this robot (`config/nav2_params.yaml`) and a nine-waypoint **patrol** mission (`patrol:=true`).
- **Record / replay**: `record:=true`, `replay.launch.py`, RViz layout, `bag_report.py`.
- **C++ mux** (`mux:=cpp`): identical to the Python mux on 4,800 random golden cases (mutation-checked); scan work 272 µs → 0.63 µs, tick 2.3 µs → 0.04 µs.
- Spawn heading argument `yaw:=`; 113 new tests (229 total).

## 1.0.0
First release: simulation, SLAM, Nav2 goals, autopilot, safety layer, hardware path, physics validation.
