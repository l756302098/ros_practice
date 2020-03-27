cmake_minimum_required(VERSION 2.8.3)
project(yd_obstacle_avoid)

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

add_library(yd_obstacle_avoid
  src/c2_algorithm.cpp
  src/velocity_smoother.cpp
  src/obstacle_detection.cpp
  src/obstacle_detection_new.cpp
  src/intelligent_plan.cpp
  src/pc2ls.cpp
)

add_dependencies(yd_obstacle_avoid ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(yd_obstacle_avoid
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

include_directories(${PROJECT_SOURCE_DIR}/include 
  ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)

#add_executable(yd_velocity_smoother src/velocity_smoother.cpp)
#target_link_libraries(yd_velocity_smoother
#  yd_obstacle_avoid
#  ${CORE_LIBRARY} 
#  ${catkin_LIBRARIES}
#)

add_executable(intelligent_plan_node src/intelligent_plan_node.cpp)
target_link_libraries(intelligent_plan_node
  yd_obstacle_avoid
  ${CORE_LIBRARY} 
  ${catkin_LIBRARIES}
)
