cmake_minimum_required(VERSION 3.0.2)
project(master_pkg)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib
  move_base_msgs
  robot_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs actionlib move_base_msgs robot_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(task_manager_node src/task_manager_node.cpp)
add_executable(delivery_node
  src/delivery_node.cpp
  src/tcp_server.cpp)


target_link_libraries(task_manager_node
  ${catkin_LIBRARIES}
)
target_link_libraries(delivery_node
  ${catkin_LIBRARIES}
)
