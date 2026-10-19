# Record and replay

```bash
ros2 launch mobile_robot_bringup launch_sim.launch.py record:=true       # bag goes to ./bags/run_<time>   (or: ros2 run mobile_robot_bringup record_bag.sh [--camera])
ros2 launch mobile_robot_bringup replay.launch.py bag:=bags/run_20261019_101500            # RViz layout for debugging, no simulator needed
ros2 launch mobile_robot_bringup replay.launch.py bag:=<folder> rate:=2.0 start:=30 loop:=true
ros2 run mobile_robot_bringup bag_report.py bags/run_20261019_101500     # numbers from the bag, no simulator
```

* `config/bag_topics.yaml`: what is recorded. *core* (a few MB/min): clock, TF, robot model, lidar, IMU, raw / fused / SLAM odometry, all command inputs, `/mux_status`, map, plan, goal.
  *camera* (about 30 MB/min) only with `--camera`. Compressed with zstd. `run.json` next to the bag stores the git revision.
* `config/bag_qos.yaml`: latched topics (`/map`, `/tf_static`, `/robot_description`) are recorded as transient-local, otherwise the replay has no map or robot model.
* `rviz/replay.rviz`: top-down floor-plan view in the `map` frame. Lidar in magenta, SLAM map, Nav2 plan, **wheel odometry trail in red and EKF-fused trail in green**
  (the slip on the wet tile shows as the two trails parting), costmaps and camera switched off by default.
* `bag_report.py`: duration, distance, minimum clearance and contacts (`/scan` + robot outline), why the mux limited the robot, and how far raw and fused odometry parted.

Replaying one bag shows the same thing every time, so a failure seen once (a stuck door, a bad localisation) can be looked at as often as needed and attached to a bug report.

Tested without ROS (`test/test_bag_tools.py`): every replayed topic is recorded, every recorded topic has a producer in the repository, the QoS file covers the latched topics,
the recorder script parses, the replay command line is right, and `bag_report` analysis on synthetic data (including the raw-vs-fused comparison).
Not tested here: `ros2 bag` itself and RViz opening the layout (needs ROS).
