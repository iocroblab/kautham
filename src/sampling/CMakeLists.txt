# libSampling

include_directories( ${mt_DIR}
# 			${Boost_INCLUDE_DIRS}
         ${CMAKE_SOURCE_DIR}/src )
#link_directories( ${CMAKE_SOURCE_DIR}/libann ${CMAKE_SOURCE_DIR}/libproblem )

file( GLOB SRC_FILES  *.cpp )
file( GLOB HDR_FILES  *.h )

set(KAUTHAM_SRC ${KAUTHAM_SRC} ${SRC_FILES} PARENT_SCOPE )

#message( STATUS "Kautham sources at ${CMAKE_CURRENT_SOURCE_DIR}: ${KAUTHAM_SRC}")

# add_library( sampling STATIC ${SRC_FILES} ${HDR_FILES} )
#
# if(UNIX)
#   set_target_properties( sampling PROPERTIES
#         LINKER_LANGUAGE CXX
#         COMPILE_FLAGS -fPIC
#         LINK_FLAGS -Wl
#         OUTPUT_NAME sampling)
# endif(UNIX)
#


#target_link_libraries( sampling libDNN ${BOOST_system_LIBRARY} )






