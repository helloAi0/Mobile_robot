#!/usr/bin/env bash
# Record a rosbag of the running robot (topics: config/bag_topics.yaml) plus a run.json that says how it was made.
#
#   ros2 run mobile_robot_bringup record_bag.sh                  # core topics -> ./bags/run_<time>
#   ros2 run mobile_robot_bringup record_bag.sh --camera         # also the camera
#   ros2 run mobile_robot_bringup record_bag.sh -o /tmp/mybag    # choose the folder
# Stop with Ctrl-C. Replay: ros2 launch mobile_robot_bringup replay.launch.py bag:=<folder>
set -eu
share="$(ros2 pkg prefix mobile_robot_bringup)/share/mobile_robot_bringup"
cfg="$share/config/bag_topics.yaml"
out="bags/run_$(date +%Y%m%d_%H%M%S)"
camera=0
while [ $# -gt 0 ]; do
  case "$1" in
    --camera) camera=1 ;;
    -o) shift; out="$1" ;;
    *) echo "usage: record_bag.sh [--camera] [-o folder]"; exit 2 ;;
  esac
  shift
done
groups="core"; [ "$camera" = 1 ] && groups="core camera"
topics=$(python3 - "$cfg" $groups <<'PY'
import sys, yaml
cfg = yaml.safe_load(open(sys.argv[1]))
print(' '.join(t for g in sys.argv[2:] for t in cfg[g]))
PY
)
mkdir -p "$(dirname "$out")"
rev=$(git -C "$share" rev-parse --short HEAD 2>/dev/null || echo unknown)
echo "recording $(echo $topics | wc -w) topics to $out  (Ctrl-C to stop)"
# the run.json is written after the bag exists (ros2 bag record creates the folder itself)
trap 'printf "{\"created\": \"%s\", \"git\": \"%s\", \"topics\": \"%s\"}\n" "$(date -Iseconds)" "$rev" "$topics" > "$out/run.json" 2>/dev/null || true' EXIT
ros2 bag record -o "$out" --compression-mode file --compression-format zstd \
  --qos-profile-overrides-path "$share/config/bag_qos.yaml" $topics
