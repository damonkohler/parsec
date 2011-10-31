cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_find_ros_package(rosserial_arduino)

set(ARDUINO_SDK_PATH /usr/share/arduino-0022)
include(${rosserial_arduino_PACKAGE_PATH}/cmake_scripts/rosserial.cmake)

set(FIRMWARE_NAME parsec)
set(${FIRMWARE_NAME}_BOARD mega2560)  # Arduino Target board
file(GLOB ${FIRMWARE_NAME}_HDRS src/*.h)
file(GLOB ${FIRMWARE_NAME}_SRCS src/*.cpp)
set(${FIRMWARE_NAME}_PORT /dev/ttyACM0)  # Serial upload port

# Uncomment the following line if you want to have the debug info on
# the base controller published
#add_definitions(-DPUBLISH_BASE_CONTROLLER_INFO)
generate_ros_firmware(${FIRMWARE_NAME})
