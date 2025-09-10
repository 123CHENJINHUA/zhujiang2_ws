cmake_minimum_required(VERSION 3.0.2)
project(navigation_pkg)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib
  move_base_msgs
  robot_msgs
  roslib
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs actionlib move_base_msgs robot_msgs roslib
  DEPENDS YAML_CPP
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_executable(navigation_node src/navigation_node.cpp)

target_link_libraries(navigation_node
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)
