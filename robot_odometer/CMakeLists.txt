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

set(SERIAL_INC_DIR /usr/local/include)
set(SERIAL_LINK_DIR /usr/local/lib/libserial.so)
include_directories(include ${catkin_INCLUDE_DIRS} ${SERIAL_INC_DIR})

###########
## Build ##
###########

add_executable(wheel_speed src/wheel_speed.cpp)
add_dependencies(wheel_speed ${catkin_EXPORTED_TARGETS} ${catkin_INCLUDE_DIRS})
target_link_libraries(wheel_speed ${catkin_LIBRARIES} ${SERIAL_LINK_DIR})

add_executable(car_odometry src/car_odometry.cpp)
add_dependencies(car_odometry ${catkin_EXPORTED_TARGETS} ${catkin_INCLUDE_DIRS})
target_link_libraries(car_odometry ${catkin_LIBRARIES} ${SERIAL_LINK_DIR})

