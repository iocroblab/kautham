#### GSL package ###############
# maths lobrary used in libproblem for the SAH hand inverse kinematics

find_package( GSL )
if( GSL_FOUND )
   message( STATUS "GSL FOUND.")
   add_definitions( -DNDEBUG )
   add_definitions( -DKAUTHAM_USE_GSL )
   include_directories( ${GSL_INCLUDE_DIR} )
   set( EXTRA_LIBS ${EXTRA_LIBS} ${GSL_LIBRARIES} ${GSL_CBLAS_LIBRARIES} PARENT_SCOPE)
else( GSL_FOUND )
   message( STATUS "GSL NOT FOUND. Hand inverse kinematics will be disabled")
endif( GSL_FOUND )


file(GLOB SRC_FILES *.c* )
file(GLOB HDR_FILES *.h*)

set(KAUTHAM_SRC ${KAUTHAM_SRC} ${SRC_FILES} PARENT_SCOPE )

# add_library(libkin STATIC ${SRC_FILES} ${HDR_FILES} )
#
# if(UNIX)
#   set_target_properties( libkin PROPERTIES
#         LINKER_LANGUAGE CXX
#         COMPILE_FLAGS -fPIC
#         LINK_FLAGS -Wl
#         OUTPUT_NAME kin)
# endif(UNIX)
#

#target_link_libraries( libkin ${EXTRA_LIBS} )
