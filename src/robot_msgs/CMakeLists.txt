cmake_minimum_required(VERSION 3.0.2)
project(robot_msgs)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  actionlib_msgs
  actionlib
  message_generation
)

add_message_files(
  FILES
  ui_show.msg
  Marker.msg
  MarkerArray.msg
  Marker2dPose.msg
  Door_open.msg
)

add_service_files(
  FILES
  delivery.srv
  ui_get.srv
  pick.srv
  unique_move.srv
  CheckMarkerId.srv
)

add_action_files(
  FILES
  delivery.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs message_runtime
)