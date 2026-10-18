cmake_minimum_required(VERSION 3.8)
project(mobile_robot_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds launch config models DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/scan_degrader.py scripts/tune_actor_height.py DESTINATION lib/${PROJECT_NAME})

ament_package()
