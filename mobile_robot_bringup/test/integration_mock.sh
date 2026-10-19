#!/usr/bin/env bash
# Integration test WITHOUT simulator or robot: starts the mock stack, drives it through the safety layer and checks
# that odometry moves. Needs a sourced ROS 2 Jazzy + built workspace.   bash test/integration_mock.sh
set -u
# ros2 launch does not always exit when it is terminated: interrupt it, give it a few seconds, then kill what is left
stop_stack() {
  kill -INT "$LP" 2>/dev/null
  for _ in 1 2 3 4 5 6; do kill -0 "$LP" 2>/dev/null || break; sleep 1; done
  kill -9 "$LP" 2>/dev/null
  pkill -9 -f "ros2_control_node|robot_state_publisher|cmd_vel_mux|auto_explore|spawner" 2>/dev/null
  wait "$LP" 2>/dev/null
}
fail() { echo "INTEGRATION TEST FAILED: $1"; stop_stack; exit 1; }
ros2 launch mobile_robot_bringup hardware.launch.py slam:=false rviz:=false > /tmp/mock_launch.log 2>&1 &
LP=$!
for i in $(seq 1 40); do
  ros2 control list_controllers 2>/dev/null | grep -q "diff_cont.*active" && break
  sleep 1
  [ "$i" = 40 ] && { tail -30 /tmp/mock_launch.log; fail "diff_cont never became active"; }
done
echo "controllers active"
ros2 topic pub -t 40 -r 20 /cmd_vel_teleop geometry_msgs/msg/Twist "{linear: {x: 0.3}}" >/dev/null 2>&1 &
sleep 3
X=$(timeout 10 ros2 topic echo --once /odom --field pose.pose.position.x 2>/dev/null | head -1)
echo "odom x = $X"
python3 -c "import sys; sys.exit(0 if float('$X') > 0.15 else 1)" 2>/dev/null || fail "robot did not move forward (odom x=$X)"
ros2 topic pub -t 5 -r 10 /emergency_stop std_msgs/msg/Bool "{data: true}" >/dev/null 2>&1
sleep 2
V=$(timeout 10 ros2 topic echo --once /diff_cont/odom --field twist.twist.linear.x 2>/dev/null | head -1)
python3 -c "import sys; sys.exit(0 if abs(float('$V')) < 0.05 else 1)" 2>/dev/null || fail "e-stop did not stop the robot (v=$V)"
echo "e-stop works (v=$V)"
stop_stack
echo "INTEGRATION TEST PASSED"
