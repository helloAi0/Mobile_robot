# Shortcuts. Every target needs ROS sourced:  source /opt/ros/jazzy/setup.bash
WS ?= $(HOME)/dev_ws
SRC := source /opt/ros/jazzy/setup.bash && source $(WS)/install/setup.bash

.PHONY: help deps build sim auto nav headless mock test pytest clean-sim
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
