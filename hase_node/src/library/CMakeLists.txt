##############################################################################
# SOURCES
##############################################################################

file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

##############################################################################
# LIBRARY
##############################################################################

add_library(hase_ros ${SOURCES})
add_dependencies(hase_ros hase_msgs_gencpp)
target_link_libraries(hase_ros ${catkin_LIBRARIES})

install(TARGETS hase_ros
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
