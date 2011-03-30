# This CMakeFile is used con create libproblem(.lib / .a)

INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/hardlab  
                     ${CMAKE_SOURCE_DIR}/libutil
                     ${CMAKE_SOURCE_DIR} 
                     ${mt_DIR} 
                     ${Boost_INCLUDE_DIRS}
                     ${COIN_INCLUDE_DIR} )

ADD_DEFINITIONS( -DCOIN_DLL )
ADD_DEFINITIONS( -DKAUTHAM_COLLISION_PQP ) # Remove it if you don't want to create a PQP collision models.

FILE( GLOB SRC_FILES *.cpp )
FILE( GLOB HDR_FILES *.h )

SET( SRC_FILES ${SRC_FILES} ${CMAKE_SOURCE_DIR}/hardlab/robot/txrobot.cpp ${CMAKE_SOURCE_DIR}/hardlab/robot/txrobot.h )

IF( GSL_FOUND )
    INCLUDE_DIRECTORIES( ${GSL_INCLUDE_DIR} )
    ADD_DEFINITIONS( -DKAUTHAM_USE_GSL )
    SET( EXTRA_LIBS ${GSL_LIBRARIES} ${GSL_CBLAS_LIBRARIES} )
ENDIF( GSL_FOUND )
  
ADD_LIBRARY(libproblem ${SRC_FILES} ${HDR_FILES})
TARGET_LINK_LIBRARIES( libproblem libPQP libsampling libkthutil libplanner ${BOOST_LIBRARIES} ${COIN_LIBRARY} ${EXTRA_LIBS} )

IF(UNIX)
 SET_TARGET_PROPERTIES( libproblem PROPERTIES OUTPUT_NAME problem )
ENDIF(UNIX)
