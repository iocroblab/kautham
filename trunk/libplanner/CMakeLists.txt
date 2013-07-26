# this file makes the libplanner library.

INCLUDE_DIRECTORIES( ${mt_DIR} 
                     ${Boost_INCLUDE_DIRS} 
                     ${CMAKE_SOURCE_DIR}  
                     ${CMAKE_SOURCE_DIR}/libutil
                     ${COIN_INCLUDE_DIR}
                     ${Armadillo_INCLUDE_DIR} )
                     
FILE( GLOB SRC_FILES *.cpp )
FILE( GLOB HDR_FILES *.h )

ADD_DEFINITIONS( -DCOIN_DLL )

#### OMPL package ###############
find_package(ompl)
IF( OMPL_FOUND )
   MESSAGE( STATUS "OMPL Package FOUND")
  SET(KC_EXTRA_OMPL_LIBS ${KC_EXTRA_OMPL_LIBS}  -lompl)
ELSE( OMPL_FOUND )
   MESSAGE( STATUS "OMPL Package NOT FOUND. OMPL planners will be disabled.")
ENDIF( OMPL_FOUND )

ADD_LIBRARY( libplanner ${SRC_FILES} ${HDR_FILES} )
TARGET_LINK_LIBRARIES( libplanner libproblem libsampling libkthutil ${COIN_LIBS} ${KC_EXTRA_OMPL_LIBS}  ${Boost_LIBRARIES})

IF(UNIX)
  SET_TARGET_PROPERTIES( libplanner PROPERTIES OUTPUT_NAME planner )
ENDIF(UNIX)
