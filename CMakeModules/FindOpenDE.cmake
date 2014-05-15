# find ODE (Open Dynamics Engine) includes and library
	#
	# ODE_INCLUDE_DIR - where the directory containing the ODE headers can be
	#                   found
	# ODE_LIBRARY     - full path to the ODE library
	# ODE_FOUND       - TRUE if ODE was found

	if (NOT ODE_FOUND)
	
	  find_path(ODE_INCLUDE_DIR ode/ode.h
	    /usr/include
	    /usr/local/include
	    $ENV{OGRE_HOME}/include # OGRE SDK on WIN32
	    $ENV{INCLUDE}
	  )
	  find_library(ODE_LIBRARY
	    NAMES ode
	    PATHS
	    /usr/lib
	    /usr/local/lib
	    $ENV{OGRE_HOME}/lib # OGRE SDK on WIN32
	  )
	
	  if(ODE_INCLUDE_DIR)
	    message(STATUS "Found ODE include dir: ${ODE_INCLUDE_DIR}")
	  else(ODE_INCLUDE_DIR)
	    message(STATUS "Could NOT find ODE headers.")
	  endif(ODE_INCLUDE_DIR)
	
	  if(ODE_LIBRARY)
	    message(STATUS "Found ODE library: ${ODE_LIBRARY}")
	  else(ODE_LIBRARY)
	    message(STATUS "Could NOT find ODE library.")
	  endif(ODE_LIBRARY)
	
	  if(ODE_INCLUDE_DIR AND ODE_LIBRARY)
	     set(ODE_FOUND TRUE CACHE STRING "Whether ODE was found or not")
	   else(ODE_INCLUDE_DIR AND ODE_LIBRARY)
	     set(ODE_FOUND FALSE)
	     if(ODE_FIND_REQUIRED)
	       message(FATAL_ERROR "Could not find ODE. Please install ODE (http://www.ode.org)")
	     endif(ODE_FIND_REQUIRED)
	   endif(ODE_INCLUDE_DIR AND ODE_LIBRARY)
	endif (NOT ODE_FOUND)
set(OMPL_EXTENSION_OPENDE ${ODE_INCLUDE_DIR} ${ODE_LIBRARY})
 #add_definitions( OPENDE_DEFINITIONS )

	# vim: et sw=4 ts=4

