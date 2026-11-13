cmake_minimum_required(VERSION 3.8)
project(ackermann_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config worlds rviz
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/manual_control.py
  DESTINATION lib/${PROJECT_NAME} RENAME manual_control)

install(PROGRAMS scripts/obstacle_spawner.py
  DESTINATION lib/${PROJECT_NAME} RENAME obstacle_spawner)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(world_map test/test_world_map.py)
  ament_add_pytest_test(obstacle_spawner test/test_obstacle_spawner.py)
endif()

ament_package()
