cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_find_ros_package(rosserial_arduino)

set(ARDUINO_SDK_PATH /usr/share/arduino-0022)
include(${rosserial_arduino_PACKAGE_PATH}/cmake_scripts/rosserial.cmake)

set(FIRMWARE_NAME parsec)
set(${FIRMWARE_NAME}_BOARD mega2560)  # Arduino Target board
set(${FIRMWARE_NAME}_HDRS src/dogs102lcd.h src/odometry.h src/parallax_ping.h src/position_controller.h src/safety.h src/shift_brite.h src/simple_led.h)
set(${FIRMWARE_NAME}_SRCS src/dogs102lcd.cpp src/parallax_ping.cpp src/parsec.cpp src/position_controller.cpp src/shift_brite.cpp src/simple_led.cpp)
set(${FIRMWARE_NAME}_PORT /dev/ttyACM0)  # Serial upload port

generate_ros_firmware(${FIRMWARE_NAME})
