cmake_minimum_required(VERSION 2.8.3)
project(hase_base)

find_package(catkin REQUIRED COMPONENTS
  controller_manager hardware_interface hase_msgs realtime_tools
  roscpp roslaunch roslint rosserial_server std_msgs)

find_package(Boost REQUIRED COMPONENTS thread chrono)

catkin_package(
  DEPENDS hase_msgs
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(hase_node
   src/hase_base.cpp
   src/hase_hardware.cpp)

target_link_libraries(hase_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(hase_node hase_msgs_gencpp rosserial_msgs_gencpp)

install(TARGETS hase_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

roslaunch_add_file_check(launch/base.launch)
roslint_cpp()
roslint_add_test()
