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
  scripts/fetch_cube.py
  scripts/gripper_command.py
  scripts/publish_pick_scene.py
  scripts/wbc_pick.py
  DESTINATION lib/${PROJECT_NAME})

install(FILES
  scripts/wbc_estimator.py
  scripts/wbc_solver.py
  scripts/wbc_task.py
  scripts/wbc_kinematics.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_wbc_pick test/test_wbc_pick.py TIMEOUT 180)
endif()

ament_package()
