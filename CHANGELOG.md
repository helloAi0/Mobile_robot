# Changelog

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
