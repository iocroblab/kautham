# libSampling

include_directories( ${mt_DIR} .. )

#link_directories( ${CMAKE_SOURCE_DIR}/libann ${CMAKE_SOURCE_DIR}/libproblem )

file( GLOB SRC_FILES  *.cpp )
file( GLOB HDR_FILES  *.h )

set(KAUTHAM_SRC ${KAUTHAM_SRC} ${SRC_FILES} PARENT_SCOPE )

