# libSampling 

INCLUDE_DIRECTORIES( ${mt_DIR} 
			${Boost_INCLUDE_DIRS} 
			${CMAKE_SOURCE_DIR}/src )
#LINK_DIRECTORIES( ${CMAKE_SOURCE_DIR}/libann ${CMAKE_SOURCE_DIR}/libproblem )

FILE( GLOB SRC_FILES  *.cpp )
FILE( GLOB HDR_FILES  *.h )

ADD_LIBRARY( sampling STATIC ${SRC_FILES} ${HDR_FILES} )

IF(UNIX)
  SET_TARGET_PROPERTIES( sampling PROPERTIES
        LINKER_LANGUAGE CXX
        COMPILE_FLAGS -fPIC
        LINK_FLAGS -Wl 
        OUTPUT_NAME sampling) 
ENDIF(UNIX)



#TARGET_LINK_LIBRARIES( sampling libDNN ${BOOST_system_LIBRARY} )





 
