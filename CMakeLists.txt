cmake_minimum_required(VERSION 3.0.2)
project(waypoint_list_server)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  tf2
  actionlib_msgs
  move_base_msgs
  actionlib
  message_generation
)
add_service_files(
  FILES
  ReadWaypointList.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(waypoint_list_server src/main.cpp)
add_executable(waypoint_vizualize src/visualize.cpp)
target_link_libraries(waypoint_list_server ${catkin_LIBRARIES})
target_link_libraries(waypoint_vizualize ${catkin_LIBRARIES})
