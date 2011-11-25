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

rosbuild_add_library(rcconsole_nodelets
  src/log_filter.cpp
  src/include_name_filter.cpp
  src/include_message_filter.cpp
  src/exclude_name_filter.cpp
  src/exclude_message_filter.cpp
  src/log_stream_printer.cpp
  src/log_file_printer.cpp)
rosbuild_link_boost(rcconsole_nodelets regex)

rosbuild_add_executable(rcconsole
  src/rcconsole.cpp
  src/rcconsole_node.cpp)
target_link_libraries(rcconsole rcconsole_nodelets)
