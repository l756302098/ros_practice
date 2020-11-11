cmake_minimum_required(VERSION 2.8.3)
project(navigation_cli)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  rospy
  std_msgs
  yidamsg
  actionlib
  actionlib_msgs
  move_base_msgs
  diagnostic_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
)
find_package(PCL 1.7 REQUIRED)

catkin_package(
   INCLUDE_DIRS
        include
   LIBRARIES
   move_base_msgs
   diagnostic_msgs
   tf2
   tf2_ros
   tf2_sensor_msgs
)

add_library(ydnavcli
  src/c2_algorithm.cpp
  src/velocity_smoother.cpp
  src/intelligent_plan.cpp
  src/Vector3.cpp
  src/pid.cpp
)

include_directories(${PROJECT_SOURCE_DIR}/include 
  ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)

add_executable(velocity_smoother src/velocity_smoother.cpp)
add_dependencies(velocity_smoother ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(velocity_smoother
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ydnavcli
)

add_executable(nav_client src/intelligent_plan_node.cpp)
add_dependencies(nav_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(nav_client
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ydnavcli
)

add_executable(pose_transfrom src/pose_transform.cpp)
add_dependencies(pose_transfrom ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pose_transfrom
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)