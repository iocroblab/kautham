# libSampling 

INCLUDE_DIRECTORIES( ${mt_DIR} ${Boost_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR} )
LINK_DIRECTORIES( ${CMAKE_SOURCE_DIR}/libann ${CMAKE_SOURCE_DIR}/libproblem )

FILE( GLOB SRC_FILES  *.cpp )
FILE( GLOB HDR_FILES  *.h )

INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/libmt 
                     ${COIN_INCLUDE_DIR} )
                     
ADD_DEFINITIONS( -DCOIN_DLL )

ADD_LIBRARY( libsampling ${SRC_FILES} ${HDR_FILES} )

TARGET_LINK_LIBRARIES( libsampling libDNN -lboost_system )





 
