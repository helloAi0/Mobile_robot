#!/usr/bin/env bash
# One-command build for the workspace that contains this repo (works from any terminal, sourced or not).
#   bash ~/dev_ws/src/mobile_robot/build.sh            # clean build
#   bash ~/dev_ws/src/mobile_robot/build.sh --keep     # incremental build (keep build/ install/ log/)
set -eo pipefail
ROS_SETUP=/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash
if [ ! -f "$ROS_SETUP" ]; then
  echo "ERROR: $ROS_SETUP not found. ROS 2 Jazzy is not installed on this machine." >&2
  echo "       Install it first: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html" >&2
  exit 1
fi
# shellcheck disable=SC1090
source "$ROS_SETUP"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$REPO/../.." && pwd)"                       # <ws>/src/mobile_robot -> <ws>
[ "$(basename "$(dirname "$REPO")")" = src ] || { echo "ERROR: expected the repo at <workspace>/src/mobile_robot, found $REPO" >&2; exit 1; }
cd "$WS"
[ "$1" = "--keep" ] || rm -rf build install log
if ! command -v colcon >/dev/null; then echo "ERROR: colcon missing: sudo apt install python3-colcon-common-extensions" >&2; exit 1; fi
colcon build --symlink-install
echo
echo "Build OK. In EVERY new terminal run:   source /opt/ros/jazzy/setup.bash && source $WS/install/setup.bash"
echo "(or add both lines to ~/.bashrc once).  Then:   ros2 launch mobile_robot_bringup launch_sim.launch.py"
