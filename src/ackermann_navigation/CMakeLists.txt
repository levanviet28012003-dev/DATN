cmake_minimum_required(VERSION 3.8)
project(ackermann_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config behavior_trees
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/trajectory_controller.py
  DESTINATION lib/${PROJECT_NAME} RENAME trajectory_controller)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(trajectory_path test/test_trajectory_path.py)
endif()

ament_package()
