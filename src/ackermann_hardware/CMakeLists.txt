cmake_minimum_required(VERSION 3.8)
project(ackermann_hardware)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/stm32_bridge.py
  DESTINATION lib/${PROJECT_NAME} RENAME stm32_bridge)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(stm32_bridge test/test_stm32_bridge.py)
endif()

ament_package()
