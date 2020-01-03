cmake_minimum_required(VERSION 2.8.3)
project(rtsp_ros)

add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED )
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(INCLUDE_DIRS include
              CATKIN_DEPENDS roscpp)


add_executable(visible_rtsp src/visible_rtsp.cpp)
add_dependencies(visible_rtsp ${catkin_EXPORTED_TARGETS} ${catkin_INCLUDE_DIRS})
target_link_libraries(visible_rtsp ${catkin_LIBRARIES}  ${OpenCV_LIBS} )
