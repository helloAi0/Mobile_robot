#!/usr/bin/env bash
# Health report for the running simulation. Open a NEW terminal (source ~/dev_ws/install/setup.bash) and run:
#   ros2 run mobile_robot diagnose.sh          # read-only checks
#   ros2 run mobile_robot diagnose.sh --move   # also drives the robot ~0.5 m forward for 2 s to test the wheels
# Copy the whole output into a message if something is [FAIL].
export PYTHONUNBUFFERED=1
MOVE=0; [ "$1" = "--move" ] && MOVE=1

ok()   { printf '  [ OK ] %s\n' "$1"; }
bad()  { printf '  [FAIL] %s\n' "$1"; FAILS=$((FAILS+1)); }
FAILS=0

has_msg() { timeout "${2:-8}" ros2 topic echo --once "$1" >/dev/null 2>&1; }

echo "== Environment"
if ros2 pkg prefix mobile_robot >/dev/null 2>&1; then ok "package mobile_robot found ($(ros2 pkg prefix mobile_robot))"
else bad "package mobile_robot not found: run 'source ~/dev_ws/install/setup.bash' in this terminal"; fi

echo "== Simulation clock"
if has_msg /clock; then ok "/clock is published (Gazebo running + clock bridge)"; else bad "no /clock: Gazebo is not running, or the bridge failed"; fi

echo "== Controllers"
CTL=$(timeout 10 ros2 control list_controllers 2>/dev/null)
echo "$CTL" | grep -q "diff_cont.*active"   && ok "diff_cont is active"   || bad "diff_cont is NOT active:$(printf '\n%s' "$CTL" | sed 's/^/        /')"
echo "$CTL" | grep -q "joint_broad.*active" && ok "joint_broad is active" || bad "joint_broad is NOT active"

echo "== Driving path (keyboard -> robot)"
INFO=$(timeout 8 ros2 topic info /diff_cont/cmd_vel 2>/dev/null)
MUX=$(timeout 8 ros2 topic info /cmd_vel_teleop 2>/dev/null | awk '/Subscription count/ {print $3}')
[ "${MUX:-0}" -ge 1 ] 2>/dev/null && ok "cmd_vel_mux is listening for the keyboard (/cmd_vel_teleop)" || bad "no cmd_vel_mux: keyboard commands go nowhere"
echo "$INFO" | grep -q "TwistStamped" && ok "/diff_cont/cmd_vel expects TwistStamped (teleop needs -p stamped:=true)" || bad "/diff_cont/cmd_vel missing or not TwistStamped:$(printf '\n%s' "$INFO" | sed 's/^/        /')"
SUBS=$(echo "$INFO" | awk '/Subscription count/ {print $3}')
[ "${SUBS:-0}" -ge 1 ] 2>/dev/null && ok "controller is listening ($SUBS subscriber)" || bad "nobody listens on /diff_cont/cmd_vel"
PUBS=$(echo "$INFO" | awk '/Publisher count/ {print $3}')
echo "        publishers right now: ${PUBS:-0} (1 while teleop is running; keys only reach it if its terminal is focused)"

echo "== Sensors"
for t in /scan /camera/image_raw /camera/depth/image_raw /camera/points /joint_states /odom /map; do
  has_msg "$t" 10 && ok "$t is publishing" || bad "$t: no message in 10 s"
done

if [ "$MOVE" = 1 ]; then
  echo "== Wheel test (drives ~0.5 m forward)"
  ros2 topic pub -r 10 -t 25 /cmd_vel_teleop geometry_msgs/msg/Twist \
     "{linear: {x: 0.3}}" >/dev/null 2>&1 &
  PUB=$!
  sleep 1.5
  VEL=$(timeout 6 ros2 topic echo --once /joint_states --field velocity 2>/dev/null | grep -Eo -- '-?[0-9]+\.?[0-9]*(e-?[0-9]+)?' | awk '{v=$1<0?-$1:$1; if (v>m) m=v} END {print m+0}')
  wait $PUB 2>/dev/null
  awk -v v="$VEL" 'BEGIN {exit !(v>0.5)}' \
    && ok "wheels turned (max wheel speed ${VEL} rad/s): controller + Gazebo are fine, the problem is the keyboard side" \
    || bad "wheels did NOT turn (max ${VEL} rad/s) although a command was sent: problem is in the controller / Gazebo physics"
fi

echo
[ "$FAILS" -eq 0 ] && echo "All checks passed." || echo "$FAILS check(s) failed."
