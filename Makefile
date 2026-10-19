# Shortcuts. Every target needs ROS sourced:  source /opt/ros/jazzy/setup.bash
WS ?= $(HOME)/dev_ws
SRC := source /opt/ros/jazzy/setup.bash && source $(WS)/install/setup.bash

.PHONY: help deps build sim auto nav headless mock test pytest clean-sim selftest physics benchmark patrol ekf record replay mux-latency
help:
	@echo "make deps     install all apt dependencies"
	@echo "make build    colcon build --symlink-install in $(WS)"
	@echo "make sim      Gazebo + RViz + SLAM, drive with the keyboard (make keys in a 2nd terminal)"
	@echo "make auto     same, the robot wanders and avoids obstacles by itself"
	@echo "make nav      same, click '2D Goal Pose' in RViz"
	@echo "make headless no Gazebo window, RViz only"
	@echo "make mock     robot software stack WITHOUT simulator or hardware (hardware.launch.py)"
	@echo "make keys     keyboard teleop     make estop / make release   emergency stop"
	@echo "make test     unit tests (no ROS needed)"
deps:
	bash mobile_robot_bringup/scripts/install_deps.sh
build:
	cd $(WS) && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install
sim:
	bash -c '$(SRC) && ros2 launch mobile_robot_bringup launch_sim.launch.py'
auto:
	bash -c '$(SRC) && ros2 launch mobile_robot_bringup launch_sim.launch.py auto:=true'
nav:
	bash -c '$(SRC) && ros2 launch mobile_robot_bringup launch_sim.launch.py nav2:=true'
headless:
	bash -c '$(SRC) && ros2 launch mobile_robot_bringup launch_sim.launch.py gui:=false'
mock:
	bash -c '$(SRC) && ros2 launch mobile_robot_bringup hardware.launch.py slam:=false'
keys:
	bash -c '$(SRC) && ros2 run mobile_robot_bringup teleop_keys.py'
estop:
	bash -c '$(SRC) && ros2 run mobile_robot_bringup estop.sh'
release:
	bash -c '$(SRC) && ros2 run mobile_robot_bringup estop.sh release'
test pytest:
	python3 -m pytest -q mobile_robot_bringup/test
clean-sim:
	-pkill -f "gz sim"; pkill -f ruby; true

selftest: ## drive test of the running simulation (no keyboard)
	bash -c '$(SRC) && ros2 run mobile_robot_bringup selftest.py'

physics: ## independent MuJoCo physics check of the robot (no ROS needed): pip install mujoco xacro numpy
	python3 mobile_robot_description/tools/physics_check.py

benchmark: ## 20 autonomous runs in the 2D model of the house, writes the README table (no ROS)
	python3 mobile_robot_bringup/tools/benchmark.py --backend sim2d --runs 20 --duration 300 --write
benchmark-gazebo: ## the same in Gazebo (needs the workspace built; slow)
	bash -c '$(SRC) && python3 mobile_robot_bringup/tools/benchmark.py --backend gazebo --runs 5 --duration 300 --out docs/benchmark_gazebo.json'
patrol: ## Nav2 patrol of all rooms
	bash -c '$(SRC) && ros2 launch mobile_robot_bringup launch_sim.launch.py nav2:=true patrol:=true'
ekf: ## simulation with wheel odometry + IMU fusion
	bash -c '$(SRC) && ros2 launch mobile_robot_bringup launch_sim.launch.py ekf:=true'
record: ## simulation that records a rosbag into ./bags
	bash -c '$(SRC) && ros2 launch mobile_robot_bringup launch_sim.launch.py record:=true'
replay: ## replay a bag in RViz:  make replay BAG=bags/run_...
	bash -c '$(SRC) && ros2 launch mobile_robot_bringup replay.launch.py bag:=$(BAG)'
mux-latency: ## Python vs C++ mux latency (needs g++, no ROS)
	python3 mobile_robot_bringup/tools/mux_latency.py
