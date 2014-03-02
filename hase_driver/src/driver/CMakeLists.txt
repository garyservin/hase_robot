##############################################################################
# SOURCES
##############################################################################

file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

##############################################################################
# CONFIGURATIONS
##############################################################################

set(VERSION_FILE ${PROJECT_SOURCE_DIR}/build/version_info.cpp)
message("making version ${hase_driver_VERSION}.")
configure_file(version_info.cpp.in ${VERSION_FILE} @ONLY)

##############################################################################
# LIBRARIES
##############################################################################

add_library(hase ${SOURCES} ${VERSION_FILE})
target_link_libraries(hase ${catkin_LIBRARIES})

install(TARGETS hase
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
