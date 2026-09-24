cmake_minimum_required(VERSION 3.5)
project(steve_manipulation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  config
  models
  DESTINATION share/${PROJECT_NAME})

install(FILES .setup_assistant
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/pick_object.py
  scripts/gripper_command.py
  scripts/publish_pick_scene.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
