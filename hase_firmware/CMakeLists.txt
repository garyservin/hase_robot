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
  TwistHaseLite.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hase_mbed
  CATKIN_DEPENDS
  geometry_msgs
  sensor_msgs
  std_msgs
  message_generation
  message_runtime
  roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(hase_mbed src/hase_mbed.cpp)
target_link_libraries(hase_mbed ${catkin_LIBRARIES})
install(TARGETS hase_mbed
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

add_executable(add_covariances src/helper/add_covariances.cpp)

target_link_libraries(add_covariances
  ${catkin_LIBRARIES}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
