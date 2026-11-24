cmake_minimum_required(VERSION 3.8)
project(ackermann_sensors)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

install(FILES README.md DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/gps_visualizer.py
  DESTINATION lib/${PROJECT_NAME} RENAME gps_visualizer)

install(PROGRAMS scripts/gps_noise_injector.py
  DESTINATION lib/${PROJECT_NAME} RENAME gps_noise_injector)

install(PROGRAMS scripts/covariance_setter.py
  DESTINATION lib/${PROJECT_NAME} RENAME covariance_setter)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(gps_visualizer test/test_gps_visualizer.py)
  ament_add_pytest_test(gps_noise_injector test/test_gps_noise_injector.py)
  ament_add_pytest_test(covariance_setter test/test_covariance_setter.py)
endif()

ament_package()
