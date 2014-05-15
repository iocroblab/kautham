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

find_path(COIN_INCLUDE_DIR Inventor/So.h
    ${CMAKE_INCLUDE_PATH}
    $ENV{COIN3DDIR}/include
    /usr/local/coin/include
    /usr/local/Coin/include
    /usr/local/include
    /usr/include
    $ENV{ProgramFiles}/Coin3D-2/include
    $ENV{COINDIR}/include
)

if(COIN_INCLUDE_DIR)
    message(STATUS "Looking for Coin3D headers -- found " ${COIN_INCLUDE_DIR}/Inventor/So.h)
    set( COIN_LIBRARY_PATH $ENV{COIN3DDIR}/lib )
else(COIN_INCLUDE_DIR )
message(SEND_ERROR 
     "Looking for Coin3D headers -- not found\n"
     "Please install Coin3D http://www.coin3d.org/ or adjust CMAKE_INCLUDE_PATH"
     "e.g. cmake -DCMAKE_INCLUDE_PATH=/path-to-Coin/include ...")
endif (COIN_INCLUDE_DIR)

find_library(COIN_LIBRARY_RELEASE
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

find_library(COIN_LIBRARY_DEBUG
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

if (COIN_LIBRARY_RELEASE)
    message(STATUS "Looking for Coin3D library -- found " ${COIN_LIBRARY_RELEASE})
else (COIN_LIBRARY_RELEASE)
    message(SEND_ERROR 
    "Looking for Coin3D library -- not found"
    "Please install Coin3D http://www.coin3d.org/ or adjust CMAKE_LIBRARY_PATH"
    "e.g. cmake -DCMAKE_LIBRARY_PATH=/path-to-Coin/lib ..."
    "or try COIN3DDIR with the correct value")
endif (COIN_LIBRARY_RELEASE)

if (COIN_LIBRARY_DEBUG)
    message(STATUS "Looking for Coin3D library debug version-- found " ${COIN_LIBRARY_DEBUG})
else (COIN_LIBRARY_DEBUG)
    message(STATUS 
    "Looking for Coin3D library with debug symbols -- not found")
endif (COIN_LIBRARY_DEBUG)


if (COIN_LIBRARY_DEBUG AND COIN_LIBRARY_RELEASE)
    # if the generator supports configuration types then set
    # optimized and debug libraries, or if the CMAKE_BUILD_TYPE has a value
    if (CMAKE_CONFIGURATION_TYPES OR CMAKE_BUILD_TYPE)
      set(COIN_LIBRARY optimized ${COIN_LIBRARY_RELEASE} debug ${COIN_LIBRARY_DEBUG})
    else(CMAKE_CONFIGURATION_TYPES OR CMAKE_BUILD_TYPE)
      # if there are no configuration types and CMAKE_BUILD_TYPE has no value
      # then just use the release libraries
      set(COIN_LIBRARY ${COIN_LIBRARY_RELEASE} )
    endif(CMAKE_CONFIGURATION_TYPES OR CMAKE_BUILD_TYPE)
    
    set(COIN_LIBRARIES optimized ${COIN_LIBRARY_RELEASE} debug ${COIN_LIBRARY_DEBUG})
else(COIN_LIBRARY_DEBUG AND COIN_LIBRARY_RELEASE)
    # if have some lib or nothing
    if(COIN_LIBRARY_DEBUG)
       set(COIN_LIBRARY ${COIN_LIBRARY_DEBUG})
    endif(COIN_LIBRARY_DEBUG)
    if(COIN_LIBRARY_RELEASE)
        set(COIN_LIBRARY ${COIN_LIBRARY_RELEASE})
    endif(COIN_LIBRARY_RELEASE)
endif (COIN_LIBRARY_DEBUG AND COIN_LIBRARY_RELEASE)

#set(COIN_LIBRARY_FOUND 1 CACHE INTERNAL "Coin3D library found")
if(COIN_INCLUDE_DIR AND COIN_LIBRARY)
    set(COIN_LIBRARY_FOUND 1 CACHE INTERNAL "Coin3D library found to develop")
endif(COIN_INCLUDE_DIR AND COIN_LIBRARY)

mark_as_advanced(
    COIN_LIBRARY_FOUND
    COIN_INCLUDE_DIR
    COIN_LIBRARY
    COIN_LIBRARY_RELEASE
    COIN_LIBRARY_DEBUG
) 
