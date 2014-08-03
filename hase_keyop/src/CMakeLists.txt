##############################################################################
# Sources
##############################################################################

file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

##############################################################################
# Targets
##############################################################################

add_executable(keyop main.cpp)
add_dependencies(keyop hase_msgs_gencpp)
target_link_libraries(keyop ${catkin_LIBRARIES})

add_executable(hase_joy_teleop teleop_joy.cpp)
target_link_libraries(hase_joy_teleop ${catkin_LIBRARIES})

install(TARGETS keyop hase_joy_teleop
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
