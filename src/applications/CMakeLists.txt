##############################################################################
# START building Kautham applications
if( KAUTHAM_GUI )
   message( STATUS "making KAUTHAM GUI" )
   add_subdirectory( graphical )
endif( KAUTHAM_GUI )

if( KAUTHAM_CONSOLE )
   message( STATUS "prepared to make KAUTHAM CONSOLE" )
   add_subdirectory( console )
endif( KAUTHAM_CONSOLE )

if( KAUTHAM_ROS )
   message( STATUS "prepared to make KAUTHAM ROS" )

SET( HDR_FILES console/kauthamshell.h )
SET( SRC_FILES ros/kautham_node.cpp  console/kauthamshell.cpp )
SET( SRC_FILES2  ros/manipulaion_node.cpp console/kauthamshell.cpp )

add_executable( Kautham_node ${SRC_FILES} ${HDR_FILES} )
TARGET_LINK_LIBRARIES( Kautham_node kautham ${catkin_LIBRARIES} ${KC_EXTRA_LIBS} ${PUGI_XML_LIBRARIES} ${BOOST_LIBRARIES} ${COIN_LIBRARY} ${EXTRA_LIBS} )


add_executable( Manipulation_node ${SRC_FILES2} ${HDR_FILES} )
TARGET_LINK_LIBRARIES( Manipulation_node kautham ${catkin_LIBRARIES} ${KC_EXTRA_LIBS} ${PUGI_XML_LIBRARIES} ${BOOST_LIBRARIES} ${COIN_LIBRARY} ${EXTRA_LIBS} )

add_executable( Manipulation_client ros/manipulation_client.cpp )
TARGET_LINK_LIBRARIES( Manipulation_client kautham ${catkin_LIBRARIES} ${KC_EXTRA_LIBS} ${PUGI_XML_LIBRARIES} ${BOOST_LIBRARIES} ${COIN_LIBRARY} ${EXTRA_LIBS} )

#########################################################
install(TARGETS Kautham_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS Manipulation_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_subdirectory( ros )
endif( KAUTHAM_ROS )
