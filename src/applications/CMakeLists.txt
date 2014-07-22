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
   #add_subdirectory( ros )

SET( HDR_FILES console/kauthamshell.h )
SET( SRC_FILES ros/kauthamshell_node.cpp console/kauthamshell.cpp )
add_executable( kauthamshell_node ${SRC_FILES} ${HDR_FILES} )
TARGET_LINK_LIBRARIES( kauthamshell_node kautham ${catkin_LIBRARIES} ${KC_EXTRA_LIBS} ${PUGI_XML_LIBRARIES} ${BOOST_LIBRARIES} ${COIN_LIBRARY} ${EXTRA_LIBS} )
endif( KAUTHAM_ROS )
