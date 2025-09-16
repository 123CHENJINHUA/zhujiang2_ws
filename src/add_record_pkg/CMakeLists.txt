cmake_minimum_required(VERSION 3.0.2)
project(add_record_pkg)

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
)

catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs
)

#############
## Install ##
#############
# set(PYTHON_EXECUTABLE /home/cjh/miniconda3/envs/zhujiang2/bin/python)

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/address_record.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)