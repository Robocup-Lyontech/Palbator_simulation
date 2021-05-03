# This package has no tests
cmake_minimum_required(VERSION 2.8.3)
project(hsrb_gazebo_bringup)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY params launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch USE_TEST_DEPENDENCIES)
endif()
