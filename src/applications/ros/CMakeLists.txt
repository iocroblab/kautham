set( HDR_FILES ../console/kauthamshell.h )
set( SRC_NODE kautham_node.cpp ../console/kauthamshell.cpp )
set( SRC_MANIPULATION_NODE  manipulaion_node.cpp ../console/kauthamshell.cpp )
set( SRC_MANIPULATION_CLIENT manipulation_client.cpp  ../console/kauthamshell.cpp)

add_executable( kautham-node ${SRC_NODE} ${HDR_FILES} )
target_link_libraries( kautham-node libkautham ${catkin_LIBRARIES}
	${KC_EXTRA_LIBS} ${PUGI_XML_LIBRARIES} ${BOOST_LIBRARIES} ${COIN_LIBRARY}
	${EXTRA_LIBS} )

add_executable( manipulation-node ${SRC_MANIPULATION_NODE} ${HDR_FILES} )
target_link_libraries( manipulation-node libkautham ${catkin_LIBRARIES}
	${KC_EXTRA_LIBS} ${PUGI_XML_LIBRARIES} ${BOOST_LIBRARIES} ${COIN_LIBRARY}
	${EXTRA_LIBS})

add_executable( manipulation-client ${SRC_MANIPULATION_CLIENT} )
target_link_libraries( manipulation-client libkautham ${catkin_LIBRARIES}
	${KC_EXTRA_LIBS} ${PUGI_XML_LIBRARIES} ${BOOST_LIBRARIES} ${COIN_LIBRARY}
	${EXTRA_LIBS})
#########################################################

#########################################################
install(TARGETS kautham-node manipulation-node manipulation-client
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

