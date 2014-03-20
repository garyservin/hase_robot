cmake_minimum_required(VERSION 2.8.3)
project(hase_keyop)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  roscpp
  rospy
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES hase
  CATKIN_DEPENDS geometry_msgs std_msgs roscpp rospy
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src)

#install(DIRECTORY launch
#        DESTINATION ${CATKIN_PACKAGE_SARE_DESTINATION}
#)
