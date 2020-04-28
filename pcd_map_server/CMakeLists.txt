cmake_minimum_required(VERSION 2.8.3)
project(pcd_map_server)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
)

catkin_package(INCLUDE_DIRS include
              CATKIN_DEPENDS roscpp)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_library(modbus
  src/modbus.cpp
  src/modbus_exception.h
)


#add_executable(map_server src/map_server.cpp)
#target_link_libraries(map_server ${catkin_LIBRARIES})
#add_dependencies(map_server ${catkin_EXPORTED_TARGETS} ${catkin_INCLUDE_DIRS})

add_executable(example src/example.cpp)
target_link_libraries(example 
${catkin_LIBRARIES}
modbus)
add_dependencies(example ${catkin_EXPORTED_TARGETS} ${catkin_INCLUDE_DIRS})