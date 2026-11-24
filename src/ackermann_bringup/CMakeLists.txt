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
  ament_add_pytest_test(worlds test/test_worlds.py)
  ament_add_pytest_test(obstacle_spawner test/test_obstacle_spawner.py)
  ament_add_pytest_test(gps_configuration test/test_gps_configuration.py)
  ament_add_pytest_test(imu_configuration test/test_imu_configuration.py)
  ament_add_pytest_test(satellite_ground test/test_satellite_ground.py)
  ament_add_pytest_test(outdoor_configuration test/test_outdoor_configuration.py)
  ament_add_pytest_test(slam_configuration test/test_slam_configuration.py)
endif()

ament_package()
