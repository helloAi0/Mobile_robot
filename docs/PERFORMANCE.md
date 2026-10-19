# C++ command mux, and what it costs

`ros2 launch mobile_robot_bringup launch_sim.launch.py mux:=cpp` (also `hardware.launch.py mux:=cpp`) runs `cmd_vel_mux_node` instead of `cmd_vel_mux.py`.
Same topics, parameters, log lines and behaviour. The Python mux stays the default and the reference.

## Design
* `include/mobile_robot_bringup/mux_core.hpp`: the whole decision logic (priority select, obstacle guard, speed and acceleration limits) as header-only C++ with no ROS.
* `src/cmd_vel_mux_node.cpp`: ROS plumbing only (parameters, QoS, subscriptions, timers).
* `ConeFinder`: finds the nearest obstacle in the front and rear cones. The beams inside a cone depend only on the scan geometry, which does not change for a real lidar,
  so they are computed once and cached: no `sin / cos / atan2` per beam per scan.

## Proof that it behaves identically (`test/test_mux_cpp.py`, `test/cpp/test_mux_golden.cpp`)
`tools/gen_mux_golden.py` draws random inputs (scans with `inf`, `nan`, zero ranges, float32 values, every cone and centre direction, thresholds on the boundary)
and records the **Python** mux's answers. The C++ test replays them: front/rear distance, guard, limits and priority selection must match to 1e-9.
It is checked that the test can fail: two deliberately broken copies of the header (`<` instead of `<=` in the guard, a wrong acceleration factor) must be rejected.
The parameter names, defaults and topics of the node are compared with the Python file by a test.

## Latency (`tools/mux_latency.py`, 720-beam scan, 20000 iterations, single thread, -O2)

| Per call | Python median | Python p99 | C++ median | C++ p99 | Speed-up (median) |
|---|---|---|---|---|---|
| Scan work: front + rear distance over 720 beams (per scan, 10 Hz) | 271.67 µs | 541.08 µs | 0.63 µs | 0.89 µs | 431x |
| Tick work: select + guard + limits (per 50 ms tick) | 2.309 µs | 6.813 µs | 0.039 µs | 0.074 µs | 59x |

Credit where it belongs: a straight C++ port **without** the cone cache takes 47 µs per scan (6x faster than Python, p99 107 µs); the other 75x of the scan speed-up comes from the cache,
an algorithmic change that Python could use too. The tick work is language only (59x).

## Read the numbers honestly
* This is **computation only**. At 10 Hz scans and 20 Hz ticks, the Python mux uses about 0.3 ms of every 100 ms, so on a desktop CPU it was never the bottleneck;
  on a Raspberry Pi-class computer, with SLAM and Nav2 competing for the cores, the headroom and the p99 tail (0.5 ms in Python) matter more.
* End-to-end latency through ROS (DDS, executor, serialisation) is **not** measured: it needs ROS, and will be dominated by the middleware, not by these microseconds.
* The node compiles in CI (`ros-build` job) and passed a syntax check against stub headers here; it has **not been run against a live ROS graph**.
  Run `mux:=cpp` in the simulator, `ros2 topic hz /diff_cont/cmd_vel` should show 20 Hz and `/mux_status` should behave as with the Python mux.
