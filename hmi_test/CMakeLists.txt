cmake_minimum_required(VERSION 2.8.3)
project(hmi_test)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hmi_test
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  src
)

set(SERIAL_SOURCES
    src/serial/serial.cc
    src/serial/impl/unix.cc
    src/serial/impl/list_ports/list_ports_linux.cc
)

add_executable(hmi_test src/main.cpp src/Uart.cpp src/hmi_ui.cpp ${SERIAL_SOURCES})
target_link_libraries(hmi_test ${catkin_LIBRARIES})