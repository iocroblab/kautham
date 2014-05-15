# this file makes the KauthamOpenDE library.

file( GLOB SRC_FILES *.cpp )
file( GLOB HDR_FILES *.h )

add_definitions( -DCOIN_DLL )

if( OMPL_FOUND )
        set(KC_EXTRA_PLANNERS_LIBS ${KC_EXTRA_PLANNERS_LIBS} ${OMPL_LIBRARIES})
       include_directories( OMPL_INCLUDE_DIRS )
else( OMPL_FOUND )
        message( STATUS "Error: OMPL Package NOT FOUND. OMPL planers will not be abaliable")
endif( OMPL_FOUND )

message( STATUS "KC_EXTRA_PLANNERS_LIBS: " ${KC_EXTRA_PLANNERS_LIBS})



add_library( libkauthamopende ${SRC_FILES} ${HDR_FILES} )
target_link_libraries( libkauthamopende libomplplanner libproblem libsampling libkthutil ${KC_EXTRA_PLANNERS_LIBS} ${Boost_LIBRARIES} ${ODE_LIBRARY})



if (TRIANGLE_FOUND)
set(OMPL_EXTENSION_TRIANGLE ${TRIANGLE_FOUND})
include_directories(SYSTEM ${TRIANGLE_INCLUDE_DIR})
endif(TRIANGLE_FOUND)



