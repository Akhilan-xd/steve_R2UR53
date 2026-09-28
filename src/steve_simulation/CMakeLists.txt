cmake_minimum_required(VERSION 3.5)
project(steve_simulation)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)

add_library(steve_grasp_attach SHARED src/grasp_attach_plugin.cpp)
ament_target_dependencies(steve_grasp_attach
  gazebo_dev
  gazebo_ros
  rclcpp
  std_srvs)

install(TARGETS steve_grasp_attach
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY launch
  configs
  maps
  worlds
  robots
  rviz
  components
  models
  DESTINATION share/${PROJECT_NAME})

ament_package()
