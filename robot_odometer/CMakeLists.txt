cmake_minimum_required(VERSION 2.8.3)
project(robot_odometer)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs
)

set(SERIAL_INC_DIR /tmp/usr/local/include)
set(SERIAL_LINK_DIR /tmp/usr/local/lib/libserial.so)
include_directories(include ${catkin_INCLUDE_DIRS} ${SERIAL_INC_DIR})

###########
## Build ##
###########

add_executable(robot_odometer src/main.cpp)
add_dependencies(robot_odometer ${catkin_EXPORTED_TARGETS} ${catkin_INCLUDE_DIRS})
target_link_libraries(robot_odometer ${catkin_LIBRARIES} ${SERIAL_LINK_DIR})
