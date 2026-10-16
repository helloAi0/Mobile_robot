# Reproducible build + test environment (CI, headless runs, hardware computer).
#   docker build -t mobile_robot .
#   docker run --rm -it --net=host mobile_robot                       # robot software in MOCK mode (no hardware, no simulator)
#   docker run --rm -it --net=host --device=/dev/robot_motor mobile_robot \
#       ros2 launch mobile_robot hardware.launch.py hardware:=real hw_plugin:=<your driver> rviz:=false
# The Gazebo window needs GPU/X11 forwarding: use the native install for the simulator demo (README).
FROM ros:jazzy-ros-base
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
      ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-rviz2 ros-jazzy-teleop-twist-keyboard \
      ros-jazzy-slam-toolbox ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-map-server \
      python3-numpy python3-yaml python3-pytest python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /ws
COPY . src/mobile_robot
RUN source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-select mobile_robot
RUN echo 'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash' >> /root/.bashrc
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && exec \"$@\"", "--"]
CMD ["ros2", "launch", "mobile_robot", "hardware.launch.py", "slam:=false", "rviz:=false"]
