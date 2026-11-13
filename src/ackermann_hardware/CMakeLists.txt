cmake_minimum_required(VERSION 3.8)
project(ackermann_hardware)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config scripts
  DESTINATION share/${PROJECT_NAME})

install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
