##############################################################################
# NODE
##############################################################################

add_library(hase_node hase_node.cpp)
add_dependencies(hase_node hase_ros)
target_link_libraries(hase_node hase_ros)

install(TARGETS hase_node
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
