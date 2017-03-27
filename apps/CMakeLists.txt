##############################################################################
# START building Kautham applications

if( KAUTHAM_IOC )
   message( STATUS "Use IOC planners: setting -DKAUTHAM_USE_IOC flag")
   add_definitions( -DKAUTHAM_USE_IOC )
endif( KAUTHAM_IOC )

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
   add_subdirectory( ros )
endif( KAUTHAM_ROS )

