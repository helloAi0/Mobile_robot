cmake_minimum_required(VERSION 3.8)
project(mobile_robot_bringup)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(unit_tests test TIMEOUT 180)
endif()

install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})

install(
  PROGRAMS scripts/object_detector.py scripts/cmd_vel_mux.py scripts/auto_explore.py scripts/teleop.sh scripts/diagnose.sh scripts/teleop_keys.py scripts/cloud_filter.py scripts/save_map.sh scripts/estop.sh scripts/install_deps.sh scripts/metrics_logger.py scripts/selftest.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
