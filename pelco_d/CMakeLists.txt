cmake_minimum_required(VERSION 2.8.3)
project(pelco_d)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

catkin_package(CATKIN_DEPENDS roscpp)
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(pelco_d src/PELCO_D.cpp src/serial.cpp)
add_dependencies(pelco_d ${catkin_EXPORTED_TARGETS})
target_link_libraries(pelco_d
     ${catkin_LIBRARIES})
