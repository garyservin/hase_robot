cmake_minimum_required(VERSION 2.8.3)
project(hase_firmware)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  message_generation
  message_runtime
  roscpp
  std_msgs
)

add_message_files(
  FILES
  ImuLite.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  sensor_msgs
  std_msgs
  message_generation
  message_runtime
  roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(add_covariances src/helper/add_covariances.cpp)

target_link_libraries(add_covariances
  ${catkin_LIBRARIES}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
