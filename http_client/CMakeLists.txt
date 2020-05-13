cmake_minimum_required(VERSION 2.8.3)
project(http_client)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(cpprestsdk_DIR /usr/lib/x86_64-linux-gnu/cmake)
find_package(cpprestsdk REQUIRED)
if (NOT cpprestsdk_FOUND)
    message(FATAL_ERROR "cpprestsdk Not Found!")
endif (NOT cpprestsdk_FOUND)

FIND_PACKAGE(CURL REQUIRED)
IF(CURL_FOUND)
message("curl Found")
ENDIF(CURL_FOUND)

find_package( OpenCV REQUIRED )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES http_client
  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(http_client src/main.cpp src/H264Decoder.cpp src/base64.cpp src/multipart_parser.cpp)
target_link_libraries(http_client ${catkin_LIBRARIES} 
  cpprestsdk::cpprest
  /usr/lib/x86_64-linux-gnu/libcurl.so
  /usr/lib/x86_64-linux-gnu/libavcodec.so
  /usr/lib/x86_64-linux-gnu/libavformat.so
  /usr/lib/x86_64-linux-gnu/libswscale.so
  /usr/lib/x86_64-linux-gnu/libavutil.so)