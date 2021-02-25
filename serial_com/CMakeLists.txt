cmake_minimum_required(VERSION 3.0.2)
project(serial_com)

add_compile_options(-std=c++11)

# Find required protobuf package
find_package(protobuf CONFIG REQUIRED)

if(protobuf_VERBOSE)
  message(STATUS "Using Protocol Buffers ${Protobuf_VERSION}")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES serial_com
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(receive_node src/receive_node.cpp src/async_serial.cpp src/urgent.pb.cc src/urgent.proto)
target_link_libraries(receive_node ${catkin_LIBRARIES}  ${Boost_LIBRARIES} protobuf::libprotobuf)

add_executable(send_node src/send_node.cpp src/async_serial.cpp src/urgent.pb.cc src/urgent.proto)
target_link_libraries(send_node ${catkin_LIBRARIES}  ${Boost_LIBRARIES} protobuf::libprotobuf)

