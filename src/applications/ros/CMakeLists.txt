set( HDR_FILES ../console/kauthamshell.h )
set( SRC_FILES kautham_node.cpp ../console/kauthamshell.cpp )
add_executable( kautham-rosnode ${SRC_FILES} ${HDR_FILES} )
target_link_libraries( kautham-rosnode libkautham ${catkin_LIBRARIES} ${PUGI_XML_LIBRARIES} )

#########################################################
install(TARGETS kautham-rosnode
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

