cmake_minimum_required(VERSION 2.8.3)
project(prov_logging)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  rosprov_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs rosprov_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/rosplan_listener.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)