ADD_SUBDIRECTORY( applications)
#ADD_SUBDIRECTORY( external ) 
ADD_SUBDIRECTORY( planner )
#ADD_SUBDIRECTORY( problem )
ADD_SUBDIRECTORY( sampling )
ADD_SUBDIRECTORY( util )


INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/src
                     ${mt_DIR}
		     ${GDIAM_DIR}
                     ${Boost_INCLUDE_DIRS}
                     ${COIN_INCLUDE_DIR}
                     ${Armadillo_INCLUDE_DIR})

IF( GSL_FOUND ) 
    SET( EXTRA_LIBS ${GSL_LIBRARIES} ${GSL_CBLAS_LIBRARIES} )
ENDIF( GSL_FOUND ) 


FILE( GLOB SRC_FILES problem/*.cpp )
FILE( GLOB HDR_FILES problem/*.h* ) 

ADD_DEFINITIONS( -DCOIN_DLL )
ADD_DEFINITIONS( -DKAUTHAM_COLLISION_PQP ) # Remove it if you don't want to create a PQP collision models.

ADD_LIBRARY(kautham SHARED ${SRC_FILES} ${HDR_FILES})

TARGET_LINK_LIBRARIES(kautham planner
			sampling
			libPQP 
			libgdiam 
			${PUGI_XML_LIBRARIES}
			${OMPL_LIBRARIES} 
			${ARMADILLO_LIBRARIES}
			${ODE_LIBRARY}
			libDNN 
			libkin
			libkthutil
			${BOOST_system_LIBRARY}
			${EXTRA_LIBS})

