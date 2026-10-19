cmake_minimum_required(VERSION 3.8)
project(mobile_robot_bringup)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(unit_tests test TIMEOUT 300)
endif()

# ---- C++ command mux (optional: needs rclcpp; the Python mux is used when it is missing) ----
find_package(rclcpp QUIET)
find_package(geometry_msgs QUIET)
find_package(nav_msgs QUIET)
find_package(sensor_msgs QUIET)
find_package(std_msgs QUIET)
if(rclcpp_FOUND AND geometry_msgs_FOUND AND nav_msgs_FOUND AND sensor_msgs_FOUND AND std_msgs_FOUND)
  add_executable(cmd_vel_mux_node src/cmd_vel_mux_node.cpp)
  target_include_directories(cmd_vel_mux_node PRIVATE include)
  target_compile_features(cmd_vel_mux_node PRIVATE cxx_std_17)
  target_compile_options(cmd_vel_mux_node PRIVATE -Wall -Wextra -Wpedantic)
  ament_target_dependencies(cmd_vel_mux_node rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs)
  install(TARGETS cmd_vel_mux_node DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING "rclcpp or message packages not found: C++ mux not built, mux:=python still works")
endif()

# ---- ROS-free C++ tests: the mux core against golden vectors from the Python mux, and the latency benchmark ----
add_executable(test_mux_golden test/cpp/test_mux_golden.cpp)
target_include_directories(test_mux_golden PRIVATE include)
target_compile_features(test_mux_golden PRIVATE cxx_std_17)
add_executable(mux_bench test/cpp/mux_bench.cpp)
target_include_directories(mux_bench PRIVATE include)
target_compile_features(mux_bench PRIVATE cxx_std_17)
install(TARGETS mux_bench DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})

install(
  PROGRAMS scripts/object_detector.py scripts/cmd_vel_mux.py scripts/auto_explore.py scripts/teleop.sh scripts/diagnose.sh scripts/teleop_keys.py scripts/cloud_filter.py scripts/save_map.sh scripts/estop.sh scripts/install_deps.sh scripts/metrics_logger.py scripts/selftest.py scripts/benchmark_metrics.py scripts/imu_covariance.py scripts/patrol.py scripts/record_bag.sh scripts/bag_report.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
