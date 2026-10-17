cmake_minimum_required(VERSION 3.8)
project(mobile_robot_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS tools/gen_mass_properties.py tools/gen_meshes.py DESTINATION lib/${PROJECT_NAME})

ament_package()
