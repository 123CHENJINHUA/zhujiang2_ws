cmake_minimum_required(VERSION 3.0.2)
project(unique_move_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  robot_msgs
)


catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs robot_msgs 
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pose_estimation_node src/pose_estimation_node.cpp)
add_executable(marker_detection_service src/marker_detection_service.cpp)

target_link_libraries(pose_estimation_node
  ${catkin_LIBRARIES}
)

target_link_libraries(marker_detection_service
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############
# set(PYTHON_EXECUTABLE /home/cjh/miniconda3/envs/zhujiang/bin/python)

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/unique_move_node.py
  scripts/test.py
  scripts/test_marker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

