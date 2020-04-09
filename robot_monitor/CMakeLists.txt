cmake_minimum_required(VERSION 2.8.3)
project(robot_monitor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  yidamsg
  velodyne_msgs
)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
)
SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(monitor src/monitor.cpp
 src/log4z.cpp)
add_dependencies(monitor ${catkin_EXPORTED_TARGETS} ${catkin_INCLUDE_DIRS} src/log4z.cpp)
target_link_libraries(monitor ${catkin_LIBRARIES})

add_executable(hdl_daemon src/hdl_daemon.cpp)
add_dependencies(hdl_daemon ${catkin_EXPORTED_TARGETS}  ${catkin_INCLUDE_DIRS})
target_link_libraries(hdl_daemon
     ${catkin_LIBRARIES})
