#!/usr/bin/env bash
# Software emergency stop for the robot (simulation and hardware). The mux holds the wheels at zero until released.
#   ros2 run mobile_robot estop.sh            # STOP now (stays stopped)
#   ros2 run mobile_robot estop.sh release    # allow driving again
# A real robot ALSO needs a physical e-stop that cuts motor power: software cannot replace it (docs/HARDWARE.md).
if [ "$1" = "release" ]; then VALUE=false; else VALUE=true; fi
exec ros2 topic pub -t 10 -r 10 /emergency_stop std_msgs/msg/Bool "{data: $VALUE}"
