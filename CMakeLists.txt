cmake_minimum_required(VERSION 3.5)
project(mobile_robot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(unit_tests test TIMEOUT 120)
endif()

install(
  DIRECTORY config description launch worlds rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/object_detector.py scripts/cmd_vel_mux.py scripts/auto_explore.py scripts/teleop.sh scripts/diagnose.sh scripts/teleop_keys.py scripts/cloud_filter.py scripts/tune_actor_height.py scripts/save_map.sh scripts/estop.sh scripts/install_deps.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
