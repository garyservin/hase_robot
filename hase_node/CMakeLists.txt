cmake_minimum_required(VERSION 2.8.3)
project(hase_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs std_msgs
  hase_driver
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hase_ros hase_node
  CATKIN_DEPENDS geometry_msgs std_msgs hase_driver
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
