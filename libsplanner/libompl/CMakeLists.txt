# this file makes the libplanner library.

FILE( GLOB SRC_FILES *.cpp )
FILE( GLOB HDR_FILES *.h )

ADD_DEFINITIONS( -DCOIN_DLL )

IF( OMPL_FOUND )
        SET(KC_EXTRA_PLANNERS_LIBS ${KC_EXTRA_PLANNERS_LIBS} -lompl)
ELSE( OMPL_FOUND )
        MESSAGE( STATUS "Error: OMPL Package NOT FOUND. OMPL planers will not be abaliable")
ENDIF( OMPL_FOUND )

MESSAGE( STATUS "KC_EXTRA_PLANNERS_LIBS: " ${KC_EXTRA_PLANNERS_LIBS})

ADD_LIBRARY( libomplplanner ${SRC_FILES} ${HDR_FILES} )
TARGET_LINK_LIBRARIES( libomplplanner libproblem libsampling libkthutil ${COIN_LIBS} ${KC_EXTRA_PLANNERS_LIBS}  ${Boost_LIBRARIES})
TARGET_LINK_LIBRARIES( libplanner libomplplanner)


