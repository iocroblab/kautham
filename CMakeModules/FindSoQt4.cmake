# Try to find SoQt library, against Qt4
# Once done this will define
#
# SOQT_LIBRARY_FOUND - If soqt is found
# SOQT_INCLUDE_DIR - where is the includes
# SOQT_LIBRARY_RELEASE - the release version
# SOQT_LIBRARY_DEBUG - the debug version
# SOQT_LIBRARY - a default library, with priority debug.
#
# This CMake finder is based in the job done 
# by Martin Wojtczyk <wojtczyk@in.tum.de> for 
# http://www.qiew.org
# 
# The modifications was done by
# Leopold Palomo <leo@alaxarxa.net>
# TODO: ensure that the soqt is linked against qt4
# with some test. Now _only_ search by the name
# works perfect with the debian version of soqt

find_path(SOQT_INCLUDE_DIR Inventor/Qt/SoQt.h
	${CMAKE_INCLUDE_PATH}
	$ENV{COIN3DDIR}/include
	/usr/local/soqt/include
	/usr/local/SoQt/include
	/usr/local/include
	/usr/include
	$ENV{ProgramFiles}/SoQt-1/include
	$ENV{ProgramFiles}/Coin3D-2/include
	$ENV{COINDIR}/include
)

if (SOQT_INCLUDE_DIR)
	message(STATUS "Looking for SoQt headers -- found " ${SOQT_INCLUDE_DIR}/Inventor/Qt/SoQt.h)
		set(SOQT_INCLUDE_DIR_FOUND 1 CACHE INTERNAL "SoQt headers found")
else (SOQT_INCLUDE_DIR)
	message(SEND_ERROR 
	"Looking for SoQt headers -- not found"
	"Please install SoQt http://www.coin3d.org/ or adjust CMAKE_INCLUDE_PATH"
	"e.g. cmake -DCMAKE_INCLUDE_PATH=/path-to-SoQt/include ...")
endif (SOQT_INCLUDE_DIR)


find_library(SOQT_LIBRARY_RELEASE
	NAMES SoQt4 soqt1 SoQt
	PATHS
	${CMAKE_LIBRARY_PATH}
	$ENV{COIN3DDIR}/lib
	/usr/local/soqt/lib
	/usr/local/SoQt/lib
	/usr/local/lib
	/usr/lib
	$ENV{ProgramFiles}/SoQt-1/lib
	$ENV{ProgramFiles}/Coin3D-2/lib
	$ENV{COINDIR}/lib
)

find_library(SOQT_LIBRARY_DEBUG
    NAMES SoQt4d SoQtd soqt1d 
    PATHS
    ${CMAKE_LIBRARY_PATH}
    $ENV{COIN3DDIR}/lib
    /usr/local/soqt/lib
    /usr/local/SoQt/lib
    /usr/local/lib
    /usr/lib/debug
    $ENV{ProgramFiles}/SoQt-1/lib
    $ENV{ProgramFiles}/Coin3D-2/lib
    $ENV{COINDIR}/lib
)

if (SOQT_LIBRARY_RELEASE)
	message(STATUS "Looking for SoQt library -- found " ${SOQT_LIBRARY_RELEASE})
else (SOQT_LIBRARY_RELEASE)
 	message(SENDL_ERROR 
	"Looking for SoQt library -- not found"
	"Please install SoQt http://www.coin3d.org/ or adjust CMAKE_LIBRARY_PATH"
	"e.g. cmake -DCMAKE_LIBRARY_PATH=/path-to-SoQt/lib ..."
    "or try COIN3DDIR with the correct value")
endif (SOQT_LIBRARY_RELEASE)

if (SOQT_LIBRARY_DEBUG)
    message(STATUS "Looking for SoQt library debug -- found " ${SOQT_LIBRARY_DEBUG})
else (SOQT_LIBRARY_DEBUG)
    message(STATUS 
    "Looking for SoQt library debug -- not found")
endif (SOQT_LIBRARY_DEBUG)


if (SOQT_LIBRARY_DEBUG AND SOQT_LIBRARY_RELEASE)
    # if the generator supports configuration types then set
    # optimized and debug libraries, or if the CMAKE_BUILD_TYPE has a value
    if (CMAKE_CONFIGURATION_TYPES OR CMAKE_BUILD_TYPE)
      set(SOQT_LIBRARY optimized ${SOQT_LIBRARY_RELEASE} debug ${SOQT_LIBRARY_DEBUG})
    else(CMAKE_CONFIGURATION_TYPES OR CMAKE_BUILD_TYPE)
      # if there are no configuration types and CMAKE_BUILD_TYPE has no value
      # then just use the release libraries
      set(SOQT_LIBRARY ${SOQT_LIBRARY_RELEASE} )
    endif(CMAKE_CONFIGURATION_TYPES OR CMAKE_BUILD_TYPE)
    
    set(SOQT_LIBRARIES optimized ${SOQT_LIBRARY_RELEASE} debug ${SOQT_LIBRARY_DEBUG})
else(SOQT_LIBRARY_DEBUG AND SOQT_LIBRARY_RELEASE)
    # if have some lib or nothing
    if(SOQT_LIBRARY_DEBUG)
       set(SOQT_LIBRARY ${SOQT_LIBRARY_DEBUG})
    endif(SOQT_LIBRARY_DEBUG)
    if(SOQT_LIBRARY_RELEASE)
        set(SOQT_LIBRARY ${SOQT_LIBRARY_RELEASE})
    endif(SOQT_LIBRARY_RELEASE)
endif (SOQT_LIBRARY_DEBUG AND SOQT_LIBRARY_RELEASE)

#set(SOQT_LIBRARY_FOUND 1 CACHE INTERNAL "Coin3D library found")
if(SOQT_INCLUDE_DIR AND SOQT_LIBRARY)
    set(SOQT_LIBRARY_FOUND 1 CACHE INTERNAL "SoQt library found, ready to use")
endif(SOQT_INCLUDE_DIR AND SOQT_LIBRARY)

mark_as_advanced(
    SOQT_LIBRARY_FOUND
    SOQT_INCLUDE_DIR
    SOQT_LIBRARY
    SOQT_LIBRARY_RELEASE
    SOQT_LIBRARY_DEBUG
) 
