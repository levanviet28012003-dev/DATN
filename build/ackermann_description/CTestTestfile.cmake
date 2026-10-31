# CMake generated Testfile for 
# Source directory: /home/viet/ackermann_ws/src/ackermann_description
# Build directory: /home/viet/ackermann_ws/build/ackermann_description
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(trajectory_path "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/viet/ackermann_ws/build/ackermann_description/test_results/ackermann_description/trajectory_path.xunit.xml" "--package-name" "ackermann_description" "--output-file" "/home/viet/ackermann_ws/build/ackermann_description/ament_cmake_pytest/trajectory_path.txt" "--command" "/usr/bin/python3" "-u" "-m" "pytest" "/home/viet/ackermann_ws/src/ackermann_description/test/test_trajectory_path.py" "-o" "cache_dir=/home/viet/ackermann_ws/build/ackermann_description/ament_cmake_pytest/trajectory_path/.cache" "--junit-xml=/home/viet/ackermann_ws/build/ackermann_description/test_results/ackermann_description/trajectory_path.xunit.xml" "--junit-prefix=ackermann_description")
set_tests_properties(trajectory_path PROPERTIES  LABELS "pytest" TIMEOUT "60" WORKING_DIRECTORY "/home/viet/ackermann_ws/build/ackermann_description" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_pytest/cmake/ament_add_pytest_test.cmake;169;ament_add_test;/home/viet/ackermann_ws/src/ackermann_description/CMakeLists.txt;28;ament_add_pytest_test;/home/viet/ackermann_ws/src/ackermann_description/CMakeLists.txt;0;")
