# this file makes the libiocplanner library.
include_directories( ${CMAKE_SOURCE_DIR}/src
                     ${mt_DIR})
file( GLOB SRC_IOC_FILES *.cpp )
file( GLOB HDR_IOC_FILES *.h )

set(KAUTHAM_SRC ${KAUTHAM_SRC} ${SRC_IOC_FILES} PARENT_SCOPE )

if( ARMADILLO_FOUND )
    message( STATUS "armadillo Package FOUND. Adding armadillo library.")
    set(EXTRA_LIBS ${EXTRA_LIBS} ${ARMADILLO_LIBRARY} PARENT_SCOPE)
    include_directories( ${ARMADILLO_INCLUDE_DIR} )
   message( STATUS "ARMADILLO_INCLUDE_DIR: " ${ARMADILLO_INCLUDE_DIR})
else( ARMADILLO_FOUND )
    message( STATUS "armadillo Package NOT FOUND. Planners using PCA will be disabled.")
endif( ARMADILLO_FOUND )

