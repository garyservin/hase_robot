cmake_minimum_required(VERSION 2.8.3)
project(hase_firmware)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  message_runtime
  rospy
  roscpp
  std_msgs
)

add_message_files(
  FILES
  OdometryLite.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_generation message_runtime rospy roscpp std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
