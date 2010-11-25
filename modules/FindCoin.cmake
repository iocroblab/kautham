# Try to find Coin3D
# Once done this will define
#
# COIN_LIBRARY_FOUND - if Coin3d is found
# COIN_INCLUDE_DIR - where are the includes
# COIN_LIBRARY_RELEASE - the relase version
# COIN_LIBRARY_DEBUG - the debug version
# COIN_LIBRARY - a default library, with priority debug.
#
# This CMake finder is based in the job done 
# by Martin Wojtczyk <wojtczyk@in.tum.de> for 
# http://www.qiew.org
# 
# The modifications was done by
# Leopold Palomo <leo@alaxarxa.net>
# TODO: 
# the coin version
# to use it with inventor?
# some test

FIND_PATH(COIN_INCLUDE_DIR Inventor/So.h
    ${CMAKE_INCLUDE_PATH}
    $ENV{COIN3DDIR}/include
    /usr/local/coin/include
    /usr/local/Coin/include
    /usr/local/include
    /usr/include
    $ENV{ProgramFiles}/Coin3D-2/include
    $ENV{COINDIR}/include
)

IF(COIN_INCLUDE_DIR)
    MESSAGE(STATUS "Looking for Coin3D headers -- found " ${COIN_INCLUDE_DIR}/Inventor/So.h)
    SET( COIN_LIBRARY_PATH $ENV{COIN3DDIR}/lib )
ELSE(COIN_INCLUDE_DIR )
MESSAGE(SEND_ERROR 
     "Looking for Coin3D headers -- not found\n"
     "Please install Coin3D http://www.coin3d.org/ or adjust CMAKE_INCLUDE_PATH"
     "e.g. cmake -DCMAKE_INCLUDE_PATH=/path-to-Coin/include ...")
ENDIF (COIN_INCLUDE_DIR)

FIND_LIBRARY(COIN_LIBRARY_RELEASE
    NAMES Coin coin2 coin3
    PATHS
    ${CMAKE_LIBRARY_PATH}
    $ENV{COIN3DDIR}/lib
    /usr/local/coin/lib
    /usr/local/lib
    /usr/lib
    $ENV{ProgramFiles}/Coin3D-2/lib
    $ENV{COINDIR}/lib
)

FIND_LIBRARY(COIN_LIBRARY_DEBUG
    NAMES Coind coin2d coin3d 
    PATHS
    ${CMAKE_LIBRARY_PATH}
    $ENV{COIN3DDIR}/lib
    /usr/local/coin/lib
    /usr/local/lib
    /usr/lib/debug
    $ENV{ProgramFiles}/Coin3D-2/lib
    $ENV{COINDIR}/lib
)

IF (COIN_LIBRARY_RELEASE)
    MESSAGE(STATUS "Looking for Coin3D library -- found " ${COIN_LIBRARY_RELEASE})
ELSE (COIN_LIBRARY_RELEASE)
    MESSAGE(SEND_ERROR 
    "Looking for Coin3D library -- not found"
    "Please install Coin3D http://www.coin3d.org/ or adjust CMAKE_LIBRARY_PATH"
    "e.g. cmake -DCMAKE_LIBRARY_PATH=/path-to-Coin/lib ..."
    "or try COIN3DDIR with the correct value")
ENDIF (COIN_LIBRARY_RELEASE)

IF (COIN_LIBRARY_DEBUG)
    MESSAGE(STATUS "Looking for Coin3D library debug version-- found " ${COIN_LIBRARY_DEBUG})
ELSE (COIN_LIBRARY_DEBUG)
    MESSAGE(STATUS 
    "Looking for Coin3D library with debug symbols -- not found")
ENDIF (COIN_LIBRARY_DEBUG)


IF (COIN_LIBRARY_DEBUG AND COIN_LIBRARY_RELEASE)
    # if the generator supports configuration types then set
    # optimized and debug libraries, or if the CMAKE_BUILD_TYPE has a value
    IF (CMAKE_CONFIGURATION_TYPES OR CMAKE_BUILD_TYPE)
      SET(COIN_LIBRARY optimized ${COIN_LIBRARY_RELEASE} debug ${COIN_LIBRARY_DEBUG})
    ELSE(CMAKE_CONFIGURATION_TYPES OR CMAKE_BUILD_TYPE)
      # if there are no configuration types and CMAKE_BUILD_TYPE has no value
      # then just use the release libraries
      SET(COIN_LIBRARY ${COIN_LIBRARY_RELEASE} )
    ENDIF(CMAKE_CONFIGURATION_TYPES OR CMAKE_BUILD_TYPE)
    
    SET(COIN_LIBRARIES optimized ${COIN_LIBRARY_RELEASE} debug ${COIN_LIBRARY_DEBUG})
ELSE(COIN_LIBRARY_DEBUG AND COIN_LIBRARY_RELEASE)
    # if have some lib or nothing
    IF(COIN_LIBRARY_DEBUG)
       SET(COIN_LIBRARY ${COIN_LIBRARY_DEBUG})
    ENDIF(COIN_LIBRARY_DEBUG)
    IF(COIN_LIBRARY_RELEASE)
        SET(COIN_LIBRARY ${COIN_LIBRARY_RELEASE})
    ENDIF(COIN_LIBRARY_RELEASE)
ENDIF (COIN_LIBRARY_DEBUG AND COIN_LIBRARY_RELEASE)

#SET(COIN_LIBRARY_FOUND 1 CACHE INTERNAL "Coin3D library found")
IF(COIN_INCLUDE_DIR AND COIN_LIBRARY)
    SET(COIN_LIBRARY_FOUND 1 CACHE INTERNAL "Coin3D library found to develop")
ENDIF(COIN_INCLUDE_DIR AND COIN_LIBRARY)

MARK_AS_ADVANCED(
    COIN_LIBRARY_FOUND
    COIN_INCLUDE_DIR
    COIN_LIBRARY
    COIN_LIBRARY_RELEASE
    COIN_LIBRARY_DEBUG
) 
