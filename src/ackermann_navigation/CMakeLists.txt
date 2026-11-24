cmake_minimum_required(VERSION 3.8)
project(ackermann_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config behavior_trees
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/gps_goal.py
  DESTINATION lib/${PROJECT_NAME} RENAME gps_goal)

install(PROGRAMS scripts/gps_waypoint_logger.py
  DESTINATION lib/${PROJECT_NAME} RENAME gps_waypoint_logger)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(gps_goal test/test_gps_goal.py)
  ament_add_pytest_test(gps_waypoints test/test_gps_waypoints.py)
endif()

ament_package()
