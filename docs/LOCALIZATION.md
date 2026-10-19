# State estimation: wheel odometry + IMU (EKF)

`ros2 launch mobile_robot_bringup launch_sim.launch.py ekf:=true` (also `hardware.launch.py ekf:=true`) runs
`robot_localization`'s `ekf_node` with `config/ekf.yaml`.

## Why it is needed here
The house has a **wet tile** in the corridor (friction 0.15, `house_world.sdf`). The wheels keep turning, the body does not follow.
Wheel odometry then reports a motion that did not happen: it integrates the slip into the pose, and the heading goes wrong,
which bends the whole SLAM map. The IMU gyro does not care about the floor: it measures how the body really turns.

## What is fused (and what is deliberately not)
| Source | Used | Not used, and why |
|---|---|---|
| wheel odometry `/diff_cont/odom` | forward speed `vx`, yaw rate `vyaw` | pose x, y, yaw: they contain the integrated slip |
| IMU `/imu/data` | gyro yaw rate | orientation (no magnetometer, simulator quaternion is world-referenced), linear acceleration (its 0.02 m/s² bias integrates to a runaway speed) |

Trust is set by covariance: wheel yaw-rate variance 2.5e-3 (`my_controllers.yaml`) against gyro variance 4e-6
(`imu_covariance.py`, from the URDF noise + turn-on bias). That is 600x more trust in the gyro for heading; a test fails if the ratio drops below 100x.
`imu_covariance.py` exists because Gazebo publishes the IMU without covariances, which a filter reads as "perfect".

Frames: the EKF owns `odom -> base_link`. With `ekf:=true` the launch files start `diff_cont` with `enable_odom_tf: false`
(`mobile_robot_gazebo/config/diff_cont_no_tf.yaml`) so two nodes never publish the same transform. The mux feeds
`/odometry/filtered` to `/odom`, which is what Nav2 and slam_toolbox read.

## Result (planar model of this configuration)
`tools/ekf_eval.py` drives across the wet tile 40 times with randomised slip and compares the two estimators with the true pose
(`tools/ekf_model.py` is the same filter as the ROS node, written in 60 lines of numpy; it reads its process noise from `ekf.yaml`):

<!-- ekf-eval:start -->
| Scenario (40 runs each) | Heading error, wheels only | Heading error, wheels + gyro (EKF) | Position error, wheels only | Position error, EKF |
|---|---|---|---|---|
| with wheel slip (wet tile) | 12.56 ± 8.52 ° | 0.96 ± 0.67 ° | 0.85 ± 0.32 m | 0.62 ± 0.27 m |
| no slip (control) | 0.53 ± 0.48 ° | 0.82 ± 0.62 ° | 0.06 ± 0.04 m | 0.07 ± 0.05 m |
<!-- ekf-eval:end -->

Read it honestly:
* **Heading** is fixed: the error after the slip drops by about 12x. That is the error that destroys maps.
* **Position** improves only a little. Slip in the *forward* direction is invisible to a gyro; the filter still believes the wheels for
  speed. The correction for that comes from SLAM (`map -> odom` from lidar scan matching), not from this filter.
* **No slip** (control row): the filter is no worse than odometry within noise. Fusing costs nothing on good floor.
* This is a model of the configuration, not the `robot_localization` node itself. The node's behaviour in Gazebo needs ROS and the simulator:
  run `ekf:=true`, drive over the wet tile, and compare in RViz with `replay.launch.py` (red = wheels, green = fused).

## Checked by tests (`test/test_ekf_config.py`, 16 tests)
sensor vectors fuse exactly `vx, vyaw` and `vyaw`; noise matrices are 15x15, diagonal, positive; frames match the controller; topics match the
producers and the Gazebo bridge; no second `odom -> base_link` publisher; gyro covariance covers the URDF noise and bias; the model keeps heading when wheels
report a turn the gyro denies, and follows the wheels when there is no gyro.
