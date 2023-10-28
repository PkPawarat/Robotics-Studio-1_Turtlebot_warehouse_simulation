# CMake generated Testfile for 
# Source directory: /home/connor/git/Robotics-Studio-1
# Build directory: /home/connor/git/Robotics-Studio-1
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_robotics-studio-1_gtest_robotics-studio-1_test "/home/connor/git/Robotics-Studio-1/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/connor/git/Robotics-Studio-1/test_results/robotics-studio-1/gtest-robotics-studio-1_test.xml" "--return-code" "/home/connor/git/Robotics-Studio-1/devel/lib/robotics-studio-1/robotics-studio-1_test --gtest_output=xml:/home/connor/git/Robotics-Studio-1/test_results/robotics-studio-1/gtest-robotics-studio-1_test.xml")
set_tests_properties(_ctest_robotics-studio-1_gtest_robotics-studio-1_test PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;98;catkin_run_tests_target;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;37;_catkin_add_google_test;/home/connor/git/Robotics-Studio-1/CMakeLists.txt;75;catkin_add_gtest;/home/connor/git/Robotics-Studio-1/CMakeLists.txt;0;")
subdirs("gtest")
