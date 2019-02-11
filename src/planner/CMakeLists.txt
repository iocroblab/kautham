# this file makes the libplanner library.
include_directories( ${mt_DIR})

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
      include_directories( OMPL_INCLUDE_DIRS )
      set ( PLANNER_EXTRA_LIBS ${OMPL_LIBRARIES} )
      message( STATUS "Adding OMPL subdirectories")

      file( GLOB SRC_OMPLG_FILES omplg/*.cpp )
      file( GLOB HDR_OMPLG_FILES omplg/*.h )
      file( GLOB SRC_OMPLC_FILES omplc/*.cpp )
      file( GLOB HDR_OMPLC_FILES omplc/*.h )
      #set(KAUTHAM_SRC ${KAUTHAM_SRC} ${SRC_OMPLG_FILES} ${SRC_OMPLC_FILES} PARENT_SCOPE  )

      if(KAUTHAM_OPENDE)
         if(ODE_FOUND)

            find_package(GLUT)
            find_package(OpenGL)

            add_definitions(-DKAUTHAM_USE_ODE)
            include_directories( ${OPENDE_INCLUDE} ${DRAWSTUFF_INCLUDE_DIR} ${OPENGL_INCLUDE_DIR} ${GLUT_INCLUDE_DIR} )
            set ( PLANNER_EXTRA_LIBS ${PLANNER_EXTRA_LIBS} ${ODE_LIBRARY}  )
            set ( PLANNER_EXTRA_LIBS ${PLANNER_EXTRA_LIBS} ${OPENDE_LIBRARY} ${DRAWSTUFF_LIBRARY} ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})
            message( STATUS "Adding subdirectory omplOpenDE")
            #file( GLOB_RECURSE SRC_OMPLOPENDE_FILES omplOpenDE/*.cpp )
            #file( GLOB_RECURSE HDR_OMPLOPENDE_FILES omplOpenDE/*.h )

            set (SRC_OMPLOPENDE_FILES 
            ${CMAKE_CURRENT_SOURCE_DIR}/omplOpenDE/InstantiatedKnowledge/Instantiatedknowledge.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/omplOpenDE/PhysicsBasedPlanners/displayOpenDE.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/omplOpenDE/PhysicsBasedPlanners/KauthamOpenDEPlanner.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/omplOpenDE/PhysicsBasedPlanners/KnowledgeOrientedKPIECE2DPlanner.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/omplOpenDE/PhysicsBasedPlanners/KnowledgeOrientedRRT2DPlanner.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/omplOpenDE/PhysicsBasedPlanners/KnowledgeOrientedSyclop2DPlanner.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/omplOpenDE/PhysicsBasedPlanners/KPIECE2DPlanner.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/omplOpenDE/PhysicsBasedPlanners/KPIECECarPlanner.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/omplOpenDE/PhysicsBasedPlanners/KPIECEChainPlanner.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/omplOpenDE/PhysicsBasedPlanners/RRT2DPlanner.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/omplOpenDE/PhysicsBasedPlanners/Syclop2DPlanner.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/omplOpenDE/Setup/CarEnvironment.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/omplOpenDE/Setup/ConstraintAware2DRobotEnvironment.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/omplOpenDE/Setup/KauthamDEGoalRegion.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/omplOpenDE/Setup/KauthamDEGoalSamplableRegion.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/omplOpenDE/Setup/KauthamOpenDEEnvironment.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/omplOpenDE/Setup/PlanarChainEnvironment.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/omplOpenDE/Setup/twoDRobotEnvironment.cpp
            )
          
            
            
            #file( GLOB SRC_OMPLOPENDE_FILES omplOpenDE/*.cpp omplOpenDE/goal/*.cpp omplOpenDE/environment/*.cpp omplOpenDE/environment/ConstraintAwareRobotEnvironment/*.cpp omplOpenDE/environment/SimpleRobotEnvironment/*.cpp
            #      omplOpenDE/InstantiatedKnowledge/*.cpp omplOpenDE/PhysicsBasedPlanners/*.cpp omplOpenDE/PhysicsBasedPlanners/PKP/*.cpp omplOpenDE/PhysicsBasedPlanners/Know-PKP/*.cpp
            #      omplOpenDE/*.h omplOpenDE/goal/*.h omplOpenDE/environment/*.h omplOpenDE/environment/ConstraintAwareRobotEnvironment/*.h omplOpenDE/environment/SimpleRobotEnvironment/*.h omplOpenDE/InstantiatedKnowledge/*.h
            #      omplOpenDE/PhysicsBasedPlanners/*.h omplOpenDE/PhysicsBasedPlanners/PKP/*.h omplOpenDE/PhysicsBasedPlanners/Know-PKP/*.h )

            #if(OMPL_EXTENSION_TRIANGLE)
            #file(GLOB SRC_OMPLOPENDE_FILES omplOpenDE/PhysicsBasedPlanners/LTL/*.cpp omplOpenDE/PhysicsBasedPlanners/LTL/*.h)
            #endif()

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

#target_link_libraries( planner ${OMPL_LIBRARIES} ${ARMADILLO_LIBRARIES} ${ODE_LIBRARY})



