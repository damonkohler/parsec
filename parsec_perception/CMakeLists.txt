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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_add_library(parsec_perception_nodelet
  src/geometry.cpp
  src/floor_filter.cpp
  src/floor_filter_nodelet.cpp
  src/laser_to_pointcloud_converter.cpp
  src/circular_robot_self_filter.cpp)

rosbuild_add_gtest(floor_filter_test test/floor_filter_test.cpp)
target_link_libraries(floor_filter_test parsec_perception_nodelet)

rosbuild_add_gtest(geometry_test test/geometry_test.cpp)
target_link_libraries(geometry_test parsec_perception_nodelet)
