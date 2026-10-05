cmake_minimum_required(VERSION 3.5)
project(steve_perception)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/cube_perception.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
