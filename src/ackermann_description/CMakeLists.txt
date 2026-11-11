cmake_minimum_required(VERSION 3.8)

project(ackermann_description)

find_package(ament_cmake REQUIRED)

install(
    DIRECTORY
        urdf
        launch
        meshes
        config
        worlds
        maps
        behavior_trees
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

install(PROGRAMS scripts/obstacle_spawner.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME obstacle_spawner
)

if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(trajectory_path test/test_trajectory_path.py)
    ament_add_pytest_test(world_map test/test_world_map.py)
    ament_add_pytest_test(obstacle_spawner test/test_obstacle_spawner.py)
endif()

ament_package()
