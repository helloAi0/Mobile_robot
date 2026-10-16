# Every command

Each new terminal needs ROS loaded first (or add both lines to `~/.bashrc` once):
```bash
source /opt/ros/jazzy/setup.bash
source ~/dev_ws/install/setup.bash
```

## Install and build (once)
```bash
cd ~/dev_ws/src && git clone -b claude/teleop-fix https://github.com/helloAi0/Mobile_robot.git mobile_robot   # or: git pull in an existing clone
bash ~/dev_ws/src/mobile_robot/scripts/install_deps.sh        # all apt packages
cd ~/dev_ws && colcon build --symlink-install && source install/setup.bash
```
Update an existing checkout and rebuild clean:
```bash
cd ~/dev_ws/src/mobile_robot && git fetch origin && git reset --hard origin/claude/teleop-fix
cd ~/dev_ws && rm -rf build/mobile_robot install/mobile_robot && colcon build --symlink-install && source install/setup.bash
```

## Simulation (Gazebo + RViz)
Kill leftovers of an earlier run first: `pkill -f "gz sim"; pkill -f ruby`

| Command | What you get |
|---|---|
| `ros2 launch mobile_robot launch_sim.launch.py` | house world, live SLAM map, drive with the keyboard |
| `... auto:=true` | the robot wanders alone, stops for obstacles and people, turns, continues, draws the map |
| `... nav2:=true` | click **2D Goal Pose** in RViz: the robot plans and drives there |
| `... world:=street_world.sdf` | outdoor street with cars, trees and walking people |
| `... world:=actor_height_test.sdf slam:=false` | 7 women at different heights (see "Walkers floating") |
| `... gui:=false` | no Gazebo window: drive and watch only in RViz |
| `... rviz:=false` / `slam:=false` / `detector:=false` / `cloud:=false` | switch parts off |
| `... x:=3.0 y:=-2.0` | spawn position |
| `... webcam:=true` | also publish your real webcam as `/webcam/image_raw` |

Arguments can be combined, e.g. `ros2 launch mobile_robot launch_sim.launch.py auto:=true gui:=false`.

## Driving
```bash
ros2 run mobile_robot teleop_keys.py          # second terminal, click into it first
```
| Key | Action |
|---|---|
| `i` `w` `Up` | forward (`u`/`o` = forward + left/right arc) |
| `m` `,` `s` `Down` | **straight backward** |
| `j` `a` `Left` / `l` `d` `Right` | turn on the spot |
| `k` `Space` `x` | stop |
| `q` / `e` | faster / slower |
| `p` | autopilot on / off |

The status line shows who drives and why the robot is limited: `teleop`, `slowing` (obstacle ahead), `obstacle` (blocked ahead: back up or turn), `obstacle-behind`, `e-stop`.

Other ways to drive: `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel` (stock teleop),
autopilot from the command line `ros2 topic pub --once /auto_mode std_msgs/msg/Bool "{data: true}"` (`false` = off).

## Safety
```bash
ros2 run mobile_robot estop.sh            # emergency stop: wheels held at zero
ros2 run mobile_robot estop.sh release    # drive again
```

## Maps
```bash
ros2 run mobile_robot save_map.sh my_house        # -> ~/maps/my_house.yaml + .pgm
```

## Checks and diagnostics
```bash
ros2 run mobile_robot diagnose.sh            # controllers, topics, sensors: [ OK ] / [FAIL]
ros2 run mobile_robot diagnose.sh --move     # also tests the wheels
ros2 control list_controllers                # diff_cont and joint_broad must be "active"
ros2 topic hz /scan                          # lidar rate (about 10 Hz)
ros2 topic echo /mux_status                  # who drives / which safety limit is active
ros2 topic echo /odom --once                 # odometry
```

## Walkers floating or sunk into the floor?
```bash
ros2 launch mobile_robot launch_sim.launch.py world:=actor_height_test.sdf slam:=false
# find the lane (n red cubes = lane n) whose woman has her feet ON the floor; z values: -0.1 0.0 0.1 0.2 0.4 0.6 0.9
python3 ~/dev_ws/src/mobile_robot/scripts/tune_actor_height.py 0.1          # women
python3 ~/dev_ws/src/mobile_robot/scripts/tune_actor_height.py 1.0 man      # men
```

## Real robot (details: docs/HARDWARE.md)
```bash
ros2 launch mobile_robot hardware.launch.py                                   # MOCK: full software stack, simulated wheels, no robot needed
ros2 launch mobile_robot hardware.launch.py hardware:=real \
     hw_plugin:=diffdrive_arduino/DiffDriveArduino hw_device:=/dev/robot_motor hw_counts_per_rev:=3436
sudo bash deploy/install_service.sh && sudo systemctl enable --now mobile_robot     # start at boot
```

## Tests
```bash
make test                                          # 59 unit tests, plain python, no ROS needed
cd ~/dev_ws && colcon test --packages-select mobile_robot && colcon test-result --verbose
bash ~/dev_ws/src/mobile_robot/test/integration_mock.sh        # mock stack: controllers, motion, e-stop (needs a built workspace)
```

## Git: push your changes
```bash
cd ~/dev_ws/src/mobile_robot          # run git ONLY here, never in ~/dev_ws
git status && git add -A && git commit -m "describe the change"
git push origin claude/teleop-fix                                   # your branch
# to publish on main:  git checkout main && git pull origin main && git merge claude/teleop-fix && git push origin main
```

## Regenerating the worlds
```bash
python3 tools/make_house_world.py        # worlds/house_world.sdf
python3 tools/make_street_world.py       # worlds/street_world.sdf
python3 tools/make_actor_test_world.py   # worlds/actor_height_test.sdf
```
