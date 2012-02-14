cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init(gmapping_offline)

rosbuild_invoke_rospack(gmapping GMAP PING find)
include_directories(${GMAP_PING}/include/gmapping)
link_directories(${GMAP_PING}/lib)

set(ROS_BUILD_TYPE Debug)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
rosbuild_add_executable(gmapping_offline src/gmapping_offline.cpp)
target_link_libraries(gmapping_offline gridfastslam sensor_odometry sensor_range utils scanmatcher)
