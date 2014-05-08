# this file makes the libplanner library.

INCLUDE_DIRECTORIES( ${mt_DIR}
                        ${Boost_INCLUDE_DIRS}
                        ${CMAKE_SOURCE_DIR}/src )

INCLUDE_DIRECTORIES( ${COIN_INCLUDE_DIR} )
ADD_DEFINITIONS( -DCOIN_DLL )


FILE( GLOB SRC_FILES *.cpp )
FILE( GLOB HDR_FILES *.h )


IF( KAUTHAM_OMPL )
    MESSAGE( STATUS "Adding OMPL subdirectories")
    
    FILE( GLOB SRC_OMPLG_FILES omplg/*.cpp )
    FILE( GLOB HDR_OMPLG_FILES omplg/*.h )
    FILE( GLOB SRC_OMPLC_FILES omplc/*.cpp )
    FILE( GLOB HDR_OMPLC_FILES omplc/*.h )

    SET(KC_PLANNERS_LIBS ${KC_PLANNERS_LIBS}  libomplplanner)
ENDIF( KAUTHAM_OMPL )
IF( KAUTHAM_IOC )
    MESSAGE( STATUS "Adding subdirectory IOC")
    FILE( GLOB SRC_IOC_FILES ioc/*.cpp )
    FILE( GLOB HDR_IOC_FILES ioc/*.h )
    SET(KC_PLANNERS_LIBS ${KC_PLANNERS_LIBS}  libiocplanner)
ENDIF( KAUTHAM_IOC )

IF( KAUTHAM_OPENDE )
    MESSAGE( STATUS "Adding subdirectory omplOpenDE")
    FILE( GLOB SRC_OMPLOPENDE_FILES omplOpenDE/*.cpp )
    FILE( GLOB HDR_OMPLOPENDE_FILES omplOpenDE/*.h )

    SET(KC_PLANNERS_LIBS ${KC_PLANNERS_LIBS}  libkauthamopende)

ENDIF( KAUTHAM_OPENDE )

#MESSAGE( STATUS "----------------KC_PLANNERS_LIBS: " ${KC_PLANNERS_LIBS})

ADD_LIBRARY( planner 	${SRC_FILES}
			${SRC_OMPLG_FILES} 
			${SRC_OMPLC_FILES} 
			${SRC_IOC_FILES}
			${SRC_OMPLOPENDE_FILES})



TARGET_LINK_LIBRARIES( planner ${KC_PLANNERS_LIBS})



