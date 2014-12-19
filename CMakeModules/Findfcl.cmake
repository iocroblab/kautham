# - Try to find FCL
# Once done, this will define
#
# FCL_FOUND - system has FCL
# FCL_INCLUDE_DIR - the FCL include directories
# FCL_LIBRARIES - link these to use FCL
FIND_PATH( FCL_INCLUDE_DIR fcl/collision.h
/usr/include
/usr/local/include
/opt/local/include
)
FIND_LIBRARY( FCL_LIBRARY fcl
/usr/lib64
/usr/lib
/usr/local/lib
/opt/local/lib
)
IF(FCL_INCLUDE_DIR AND FCL_LIBRARY)
SET( FCL_FOUND TRUE )
SET( FCL_LIBRARIES ${FCL_LIBRARY} )
ENDIF(FCL_INCLUDE_DIR AND FCL_LIBRARY)
IF(FCL_FOUND)
IF(NOT FCL_FIND_QUIETLY)
MESSAGE(STATUS "Found fcl: ${FCL_LIBRARY}")
ENDIF(NOT FCL_FIND_QUIETLY)
ELSE(FCL_FOUND)
IF(FCL_FIND_REQUIRED)
MESSAGE(FATAL_ERROR "Could not find fcl")
ENDIF(FCL_FIND_REQUIRED)
ENDIF(FCL_FOUND)
