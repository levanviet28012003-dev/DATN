cmake_minimum_required(VERSION 3.8)
project(ackermann_sensors)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

install(FILES README.md DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/gps_visualizer.py
  DESTINATION lib/${PROJECT_NAME} RENAME gps_visualizer)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(gps_visualizer test/test_gps_visualizer.py)
endif()

ament_package()
