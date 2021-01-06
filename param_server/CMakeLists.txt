cmake_minimum_required(VERSION 3.0.2)
project(param_server)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslib
  message_generation
)
find_package(Boost REQUIRED COMPONENTS system)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_message_files(
  FILES
  KeyValue.msg
  KeyValues.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Param.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES param_server
  CATKIN_DEPENDS roscpp std_msgs roslib
)

add_executable(param_test src/test.cpp src/server.cpp)
target_link_libraries(param_test ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES} yaml-cpp)

add_library(param_server
  src/server.cpp)
target_link_libraries(param_server ${Boost_LIBRARIES} yaml-cpp)

install(DIRECTORY include/param_server/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")

install(TARGETS param_server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})