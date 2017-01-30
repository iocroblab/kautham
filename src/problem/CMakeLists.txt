# This CMakeFile is used con create libproblem(.lib / .a)

include_directories( ${GDIAM_DIR} )

include_directories( ..
                     ${mt_DIR})

if( ASSIMP_FOUND )
   include_directories( ${ASSIMP_INCLUDE_DIR})
endif( ASSIMP_FOUND )

if( FCL_FOUND )
   include_directories( ${FCL_INCLUDE_DIR})
endif( FCL_FOUND )

if( ARMADILLO_FOUND )
   include_directories( ${Armadillo_INCLUDE_DIR})
endif( ARMADILLO_FOUND )

file( GLOB SRC_FILES *.cpp )
file( GLOB HDR_FILES *.h* )

set(KAUTHAM_SRC ${KAUTHAM_SRC} ${SRC_FILES} PARENT_SCOPE )
#add_library(problem STATIC ${SRC_FILES} ${HDR_FILES})
#target_link_libraries(problem libPQP libgdiam planner sampling ${PUGI_XML_LIBRARIES})


