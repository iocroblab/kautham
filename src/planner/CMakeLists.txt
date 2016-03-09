# this file makes the libplanner library.
include_directories( ${CMAKE_SOURCE_DIR}/src
                     ${mt_DIR})

file( GLOB SRC_FILES *.cpp )
file( GLOB HDR_FILES *.h )

if( KAUTHAM_IOC )
   message( STATUS "Adding subdirectory IOC")
   add_subdirectory( ioc )
endif( KAUTHAM_IOC )



##############################################################################
##############################################################################
# START Required packages when using OMPL planners
if( KAUTHAM_OMPL )
    #### OMPL package ###############

   if( OMPL_FOUND )
      message( STATUS "OMPL Package FOUND: setting -DKAUTHAM_USE_OMPL flag")
      message( STATUS "OMPL lib is: " ${OMPL_LIBRARIES})
      message( STATUS "OMPL include is: " ${OMPL_INCLUDE_DIRS})
      include_directories( OMPL_INCLUDE_DIRS )
      set ( PLANNER_EXTRA_LIBS ${OMPL_LIBRARIES} )
      message( STATUS "OMPL VERSION is: " ${OMPL_VERSION})
      message( STATUS "Adding OMPL subdirectories")

      file( GLOB SRC_OMPLG_FILES omplg/*.cpp )
      file( GLOB HDR_OMPLG_FILES omplg/*.h )
      file( GLOB SRC_OMPLC_FILES omplc/*.cpp )
      file( GLOB HDR_OMPLC_FILES omplc/*.h )
      #set(KAUTHAM_SRC ${KAUTHAM_SRC} ${SRC_OMPLG_FILES} ${SRC_OMPLC_FILES} PARENT_SCOPE  )

      if(KAUTHAM_OPENDE)
         if(ODE_FOUND)
            message( STATUS "OMPL Package FOUND: setting -DKAUTHAM_USE_ODE flag")
            include_directories( ${ODE_INCLUDE_DIR} )
            set ( PLANNER_EXTRA_LIBS ${PLANNER_EXTRA_LIBS} ${ODE_LIBRARY}  )
            message( STATUS "Adding subdirectory omplOpenDE")
            #file( GLOB SRC_OMPLOPENDE_FILES omplOpenDE/*.cpp )
            #file( GLOB HDR_OMPLOPENDE_FILES omplOpenDE/*.h )

            file( GLOB SRC_OMPLOPENDE_FILES omplOpenDE/*.cpp omplOpenDE/goal/*.cpp omplOpenDE/environment/*.cpp omplOpenDE/environment/ConstraintAwareRobotEnvironment/*.cpp omplOpenDE/environment/SimpleRobotEnvironment/*.cpp
                  omplOpenDE/InstantiatedKnowledge/*.cpp omplOpenDE/PhysicsBasedPlanners/*.cpp omplOpenDE/PhysicsBasedPlanners/PKP/*.cpp omplOpenDE/PhysicsBasedPlanners/Know-PKP/*.cpp omplOpenDE/PhysicsBasedPlanners/LTL/*.cpp
		  omplOpenDE/*.h omplOpenDE/goal/*.h omplOpenDE/environment/*.h omplOpenDE/environment/ConstraintAwareRobotEnvironment/*.h omplOpenDE/environment/SimpleRobotEnvironment/*.h omplOpenDE/InstantiatedKnowledge/*.h 
                  omplOpenDE/PhysicsBasedPlanners/*.h omplOpenDE/PhysicsBasedPlanners/PKP/*.h omplOpenDE/PhysicsBasedPlanners/Know-PKP/*.h omplOpenDE/PhysicsBasedPlanners/LTL/*.h)
 
#	    file( GLOB SRC_OMPLOPENDE_FILES omplOpenDE/goal/*.h omplOpenDE/environment/*.h omplOpenDE/environment/CRE/*.h omplOpenDE/environment/RE/*.h omplOpenDE/InstantiatedKnowledge/*.h 			  omplOpenDE/PhysicsBasedPlanners/*.h omplOpenDE/PhysicsBasedPlanners/PKP/*.h omplOpenDE/PhysicsBasedPlanners/Know-PKP/*.h)

 	    #file( GLOB HDR_OMPLOPENDE_FILES omplOpenDE/goal/*.cpp )
            #file( GLOB HDR_OMPLOPENDE_FILES omplOpenDE/goal/*.h )

 	    #file( GLOB SRC_OMPLOPENDE_FILES omplOpenDE/environment/*.cpp )
            #file( GLOB HDR_OMPLOPENDE_FILES omplOpenDE/environment/*.h )
 	    #file( GLOB SRC_OMPLOPENDE_FILES omplOpenDE/environment/CRE/*.cpp )
            #file( GLOB HDR_OMPLOPENDE_FILES omplOpenDE/environment/CRE/*.h )
 	    #file( GLOB SRC_OMPLOPENDE_FILES omplOpenDE/environment/RE/*.cpp )
            #file( GLOB HDR_OMPLOPENDE_FILES omplOpenDE/environment/RE/*.h )				
 	    #file( GLOB SRC_OMPLOPENDE_FILES omplOpenDE/InstantiatedKnowledge/*.cpp )
            #file( GLOB HDR_OMPLOPENDE_FILES omplOpenDE/InstantiatedKnowledge/*.h )
 	    #file( GLOB SRC_OMPLOPENDE_FILES omplOpenDE/PhysicsBasedPlanners/*.cpp)
            #file( GLOB HDR_OMPLOPENDE_FILES omplOpenDE/PhysicsBasedPlanners/*.h )
 	    #file( GLOB SRC_OMPLOPENDE_FILES omplOpenDE/PhysicsBasedPlanners/PKP/*.cpp )
            #file( GLOB HDR_OMPLOPENDE_FILES omplOpenDE/PhysicsBasedPlanners/PKP/*.h )
 	    #file( GLOB SRC_OMPLOPENDE_FILES omplOpenDE/PhysicsBasedPlanners/Know-PKP/*.cpp )
            #file( GLOB HDR_OMPLOPENDE_FILES omplOpenDE/PhysicsBasedPlanners/Know-PKP/*.h )

    	 else( ODE_FOUND )
	      message( STATUS "Error: ODE LIBRARY NOT FOUND. OMPL planner with ODE will not be avaliable")
         endif(ODE_FOUND)
      endif(KAUTHAM_OPENDE)

    else( OMPL_FOUND )
      message( STATUS "Error: OMPL Package NOT FOUND. OMPL planers will not be avaliable")
    endif( OMPL_FOUND )
endif( KAUTHAM_OMPL )

set(KAUTHAM_SRC ${KAUTHAM_SRC}
               ${SRC_FILES}
               ${SRC_OMPLOPENDE_FILES}
               ${SRC_OMPLG_FILES}
               ${SRC_OMPLC_FILES}
               PARENT_SCOPE )

set ( EXTRA_LIBS ${EXTRA_LIBS} ${PLANNER_EXTRA_LIBS} PARENT_SCOPE )

# add_library( planner STATIC ${SRC_FILES}
# 			${SRC_OMPLG_FILES}
# 			${SRC_OMPLC_FILES}
# 			${SRC_IOC_FILES}
# 			${SRC_OMPLOPENDE_FILES})
#
# if(UNIX)
#   set_target_properties( planner PROPERTIES
#         LINKER_LANGUAGE CXX
#         COMPILE_FLAGS -fPIC
#         LINK_FLAGS -Wl
#         OUTPUT_NAME planner)
# endif(UNIX)
#

#if( KAUTHAM_GUIBRO )
#    message( STATUS "Use GUIBRO: setting -DKAUTHAM_USE_GUIBRO flag")
#    add_definitions( -DKAUTHAM_USE_GUIBRO )
#endif( KAUTHAM_GUIBRO )


#target_link_libraries( planner ${OMPL_LIBRARIES} ${ARMADILLO_LIBRARIES} ${ODE_LIBRARY})



