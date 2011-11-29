cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_executable(priority_mux
  src/priority_mux.cpp
  src/expiring_subscription.cpp
  src/priority_mux_node.cpp)

rosbuild_add_gtest(expiring_subscription_test
  src/expiring_subscription.cpp
  test/expiring_subscription_test.cpp)

rosbuild_add_gtest(priority_mux_test
  src/priority_mux.cpp
  src/expiring_subscription.cpp
  test/priority_mux_test.cpp)
