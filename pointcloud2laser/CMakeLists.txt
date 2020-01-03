cmake_minimum_required(VERSION 2.8.3)
project(pointcloud2laser)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  laser_geometry
  message_filters
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  geometry_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(INCLUDE_DIRS include
              CATKIN_DEPENDS std_msgs roscpp laser_geometry message_filters sensor_msgs tf2 tf2_ros tf2_sensor_msgs geometry_msgs)

add_executable(pointcloud2laser src/main.cpp)
add_dependencies(pointcloud2laser ${catkin_EXPORTED_TARGETS} ${catkin_INCLUDE_DIRS})
target_link_libraries(pointcloud2laser ${catkin_LIBRARIES})
