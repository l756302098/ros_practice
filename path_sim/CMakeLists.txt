cmake_minimum_required(VERSION 2.8.3)
project(path_sim)

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
)
find_package(PCL 1.7 REQUIRED)

#add_action_files(DIRECTORY action FILES YDPathPlanning.action)
#generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
   LIBRARIES
   stp
   move_base_msgs
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

add_library(c3_algorithm
src/c3_algorithm.cpp
src/c2_algorithm.cpp
)


include_directories(${PROJECT_SOURCE_DIR}/include 
${catkin_INCLUDE_DIRS} 
${PCL_INCLUDE_DIRS}
)


add_executable(plan_sim src/plan_sim.cpp src/Vect.cpp src/utils.cpp)
target_link_libraries(plan_sim
    ${CORE_LIBRARY} 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

add_executable(path_planning src/path_planning.cpp src/Vect.cpp)
target_link_libraries(path_planning
    ${CORE_LIBRARY} 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

add_executable(velocity_smoother src/velocity_smoother.cpp)
add_dependencies(velocity_smoother ${catkin_EXPORTED_TARGETS} ${catkin_INCLUDE_DIRS})
target_link_libraries(velocity_smoother
    c3_algorithm
    ${CORE_LIBRARY} 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

#add_executable(loop_planning_client src/loop_planning_client.cpp)
#target_link_libraries(loop_planning_client
#    ${CORE_LIBRARY} 
#    ${catkin_LIBRARIES}
#    ${PCL_LIBRARIES}
#)

#add_executable(loop_planning_server src/loop_planning_server.cpp)
#target_link_libraries(loop_planning_server
#    ${CORE_LIBRARY} 
#    ${catkin_LIBRARIES}
#    ${PCL_LIBRARIES}
#)

add_executable(obstacle_detection src/obstacle_detection.cpp)
target_link_libraries(obstacle_detection
    ${CORE_LIBRARY} 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

add_executable(obstacle_avoid_planning src/obstacle_avoid_planning.cpp)
target_link_libraries(obstacle_avoid_planning
    ${CORE_LIBRARY} 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)