# From simulation to a real robot

The software stack is the same in both worlds (see ARCHITECTURE.md). On hardware you provide **two drivers**; everything else is already here.

| You provide | Must publish / offer | Where it plugs in |
|---|---|---|
| **Motor driver** (a `ros2_control` hardware interface) | wheel velocity command interface, position + velocity state interfaces for `left_wheel_joint`, `right_wheel_joint` | `mobile_robot_description/urdf/hardware_control.xacro`, launch argument `hw_plugin:=` |
| **Lidar driver** | `/scan` (`sensor_msgs/LaserScan`), frame `laser_frame` | its own launch file; the frame position is set in `mobile_robot_description/urdf/lidar.xacro` |
| Camera (optional) | `/camera/image_raw` etc. | `webcam.launch.py` (USB) or your driver |

Typical parts: a differential-drive base with encoders, a serial motor board (Arduino/ESP32/ODrive/RoboClaw...), a 2D lidar (RPLIDAR / LDROBOT / YDLIDAR),
an onboard computer with Ubuntu 24.04 + ROS 2 Jazzy (Raspberry Pi 5, Jetson Orin...).

## Step 1: test the whole software stack with NO hardware (mock)
```bash
ros2 launch mobile_robot_bringup hardware.launch.py slam:=false
```
Starts robot_state_publisher, ros2_control with simulated wheels (`mock_components/GenericSystem`), controllers, mux and safety layer.
In a second terminal `ros2 run mobile_robot_bringup teleop_keys.py`, then `ros2 topic echo /odom`: it moves. This is also what CI runs (`test/integration_mock.sh`).

## Step 2: motor driver
1. Install a `ros2_control` diff-drive hardware interface for your motor board (example for Arduino-style serial boards: `diffdrive_arduino`, branch `jazzy`; for ODrive/CAN boards use their ros2_control package). The driver must expose the joints `left_wheel_joint` and `right_wheel_joint` with a `velocity` command interface and `position`/`velocity` state interfaces.
2. Edit the `<param>` lines in `mobile_robot_description/urdf/hardware_control.xacro` to the names your driver expects (device, baud rate, encoder counts, loop rate...). The launch arguments `hw_device`, `hw_baud`, `hw_counts_per_rev` fill them in.
3. **Wheels off the ground first**:
```bash
ros2 launch mobile_robot_bringup hardware.launch.py hardware:=real hw_plugin:=diffdrive_arduino/DiffDriveArduino \
     hw_device:=/dev/robot_motor hw_counts_per_rev:=3436 slam:=false rviz:=false
ros2 run mobile_robot_bringup teleop_keys.py        # lift the robot: do both wheels turn the right way?  ros2 topic echo /joint_states
```
Wrong direction? Swap the motor wires or add `<param name="invert">` as your driver supports.

## Step 3: calibrate (in `mobile_robot_bringup/config/my_controllers.yaml`)
| Parameter | How |
|---|---|
| `wheel_radius` | drive a marked 2.0 m with the keyboard; `ros2 topic echo /odom`: new radius = old × (real distance / odom distance) |
| `wheel_separation` | rotate in place 10 full turns (3600°) with `j`; if odom says more/less than 3600° scale the separation the opposite way |
| `cmd_vel_timeout` | keep 0.5 s: the robot stops by itself if the command stream stops |
| speed limits | `max_linear`, `max_angular`, `max_accel` of `cmd_vel_mux` (`ros2 param set` or a params file); start slow (0.2 m/s) |

Lidar mount: measure its position from the middle of the wheel axle (`base_link`) and edit `laser_joint` in `mobile_robot_description/urdf/lidar.xacro`. Check in RViz that walls appear straight and in the right place while the robot turns.

## Step 4: lidar and SLAM
Start your lidar driver so that `/scan` publishes (check: `ros2 topic hz /scan`). Then:
```bash
ros2 launch mobile_robot_bringup hardware.launch.py hardware:=real hw_plugin:=... slam:=true rviz:=false
rviz2 -d $(ros2 pkg prefix mobile_robot_bringup)/share/mobile_robot_bringup/rviz/camera.rviz        # on your laptop (same ROS_DOMAIN_ID, same network)
```
Drive slowly through the room, close loops, then `ros2 run mobile_robot_bringup save_map.sh lab`.
Add `nav2:=true` or `auto:=true` once the map looks right. Increase `safety_stop_distance` for bigger robots (`ros2 param set /cmd_vel_mux safety_stop_distance 0.5`).

## Step 5: run at boot
```bash
sudo bash deploy/install_service.sh                 # systemd service + udev rules (stable names /dev/robot_motor, /dev/robot_lidar)
sudo nano /etc/default/mobile_robot                 # your hw_plugin and arguments
sudo systemctl enable --now mobile_robot && journalctl -u mobile_robot -f
```

## Safety (read before the first drive)
* The software e-stop (`ros2 run mobile_robot_bringup estop.sh`) and the lidar guard are **not** a replacement for a **physical emergency stop that cuts motor power**. Fit one before the robot drives near people.
* Test on blocks, then at 0.2 m/s in an empty area, then increase.
* The guard only sees what the 2D lidar sees (one horizontal plane): low obstacles, glass, stairs/drop-offs and anything above the lidar plane are invisible. Add cliff/bumper sensors for real environments.
* The stack stops by itself when the lidar stops (no fresh `/scan` = no forward motion) and when drivers stop sending (0.5 s timeouts).

## Production checklist
| Item | Status in this repository |
|---|---|
| Same software in sim and on robot | yes (hardware.launch.py, mock mode) |
| Safety layer: e-stop, obstacle guard front/rear, fail-safe on sensor loss, speed/accel limits, watchdogs | yes, unit-tested (`test/test_mux_safety.py`) |
| Automated tests + CI | 116 tests, xacro checks for sim/mock/real, mock-hardware integration test in CI |
| Reproducible environment | `mobile_robot_bringup/scripts/install_deps.sh`, `Dockerfile` |
| Start at boot, device naming | `deploy/` (systemd, udev) |
| Motor and lidar drivers for YOUR parts | **you provide** (Step 2 / 4) |
| Hardware validation of SLAM/Nav2 tuning (footprint, costmap inflation, speeds) | **to do on the robot**; Nav2 runs with nav2_bringup defaults (robot radius 0.22 m) |
| Localization on a saved map (AMCL) | not included yet: the stack maps while it drives; `slam_toolbox` localization mode with `map_file_name` is the next step |
| Battery monitoring, diagnostics aggregator, bag recording | not included yet |
| Functional-safety certification | out of scope; needs a hardware safety chain and a formal assessment |

Everything marked "yes" was exercised in unit tests and static checks; the Gazebo and ROS-runtime behaviour must be confirmed on your machine (see DEMO.md for a 10-minute acceptance run).
