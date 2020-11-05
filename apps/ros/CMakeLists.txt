file(GLOB SRC_FILES *.c* )
file(GLOB HDR_FILES *.h*)
#set( HDR_FILES ../../include/kautham/kauthamshell.h )
#set( SRC_FILES kautham_node.cpp ../../src/kauthamshell.cpp )

add_executable( kautham-rosnode kautham_node.cpp )
target_link_libraries( kautham-rosnode libkautham ${catkin_LIBRARIES} ${PUGI_XML_LIBRARIES} )

add_executable( kautham-rosnode-vis kautham_node_vis.cpp )
target_link_libraries( kautham-rosnode-vis libkautham ${catkin_LIBRARIES} ${PUGI_XML_LIBRARIES} )

#########################################################
install(TARGETS kautham-rosnode kautham-rosnode-vis
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

