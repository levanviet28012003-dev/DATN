cmake_minimum_required(VERSION 3.8)

project(ackermann_description)

find_package(ament_cmake REQUIRED)

install(
    DIRECTORY
        urdf
        launch
        meshes
        config
    DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/manual_control.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME manual_control
)

install(PROGRAMS scripts/trajectory_controller.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME trajectory_controller
)

if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(trajectory_path test/test_trajectory_path.py)
endif()

ament_package()
