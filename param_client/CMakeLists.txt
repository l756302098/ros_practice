cmake_minimum_required(VERSION 3.0.2)
project(param_client)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  param_server
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES param_client 
  CATKIN_DEPENDS roscpp std_msgs param_server
#  DEPENDS system_lib
)

add_executable(client src/client.cpp)
target_link_libraries(client ${catkin_LIBRARIES})