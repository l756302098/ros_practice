cmake_minimum_required(VERSION 2.8.3)
project(message_sync)

add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED )
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  sensor_msgs
  message_filters
  yidamsg
  cv_bridge
  image_transport
  Eigen3
)

add_message_files(
FILES
  pointcloud_color.msg
  pointcloud_color2.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

catkin_package(INCLUDE_DIRS include
              CATKIN_DEPENDS roscpp)

#add_executable(message_sync src/message_sync.cpp  src/base64.cpp)
#add_dependencies(message_sync ${catkin_EXPORTED_TARGETS} ${catkin_INCLUDE_DIRS})
#target_link_libraries(message_sync ${catkin_LIBRARIES})

#add_executable(sync_thermal src/sync_thermal.cpp src/base64.cpp)
##add_dependencies(sync_thermal ${catkin_EXPORTED_TARGETS} ${catkin_INCLUDE_DIRS})
#target_link_libraries(sync_thermal ${catkin_LIBRARIES})

add_executable(sync_visible src/sync_visible.cpp src/base64.cpp)
add_dependencies(sync_visible ${catkin_EXPORTED_TARGETS} ${catkin_INCLUDE_DIRS})
target_link_libraries(sync_visible ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})

add_executable(sync_imu src/sync_imu.cpp)
add_dependencies(sync_imu ${catkin_EXPORTED_TARGETS} ${catkin_INCLUDE_DIRS})
target_link_libraries(sync_imu ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})

add_executable(save_message src/save_message.cpp src/base64.cpp)
add_dependencies(save_message ${catkin_EXPORTED_TARGETS} ${catkin_INCLUDE_DIRS})
target_link_libraries(save_message ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})