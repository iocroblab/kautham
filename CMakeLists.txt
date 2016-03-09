project(kautham2)

cmake_minimum_required(VERSION 2.4.6)


if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)
    endif(COMMAND cmake_policy)


########### Options ####################
option( KAUTHAM_OMPL "Use OMPL planners" ON )
option( KAUTHAM_IOC "Use IOC planners" ON )
#pending job
#option( KAUTHAM_GUIBRO "Use Guibro planner (virtual bronchoscopies)" OFF)
option( KAUTHAM_OPENDE "Use Open Dynamic Engine" ON)
option( KAUTHAM_GUI "Build the gui version" ON )
option( KAUTHAM_CONSOLE "Build the console version" ON )
option( KAUTHAM_ROS "Build the ROS version" ON )
option( KAUTHAM_DOCUMENTATION "Create and install the HTML based API documentation (requires Doxygen)" ON)

########### Set flags ####################
message( STATUS "Using this flags ........")
message( STATUS "Building the GUI version: ${KAUTHAM_GUI}")
message( STATUS "Building the console version: ${KAUTHAM_CONSOLE}")
message( STATUS "Building the ROS version: ${KAUTHAM_ROS}")
message( STATUS "Using OMPL planners: ${KAUTHAM_OMPL}")
message( STATUS "Using IOC planners: ${KAUTHAM_IOC}")
#message( STATUS "Using Guibro planner (virtual bronchoscopies) ${KAUTHAM_GUIBRO}")
message( STATUS "Using Open Dynamic Engine: ${KAUTHAM_OPENDE}")
message( STATUS "Creating the HTML based API documentation: ${KAUTHAM_DOCUMENTATION}")

########### Modules path ###############
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)
add_definitions ( -Wall )
##############################################################################


##################### ROS-catkin ####################################


if( KAUTHAM_ROS )
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  trajectory_msgs
  message_generation
)
FIND_PACKAGE( Boost COMPONENTS system serialization thread REQUIRED )

INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

include_directories(include ${catkin_INCLUDE_DIRS})
 LINK_DIRECTORIES(${Boost_LIBRARY_DIRS} )

add_service_files(
   DIRECTORY srv
   FILES 
   AddObstacle.srv
   DetachObstacle.srv
   SetDefaultRobControls.srv
   SetObsControlsStream.srv
   SetQuery.srv
   AddRobot.srv
   GetPath.srv
   SetFixedObsControls.srv
   SetObstaclesConfig.srv
   SetRobControls.srv
   AttachObstacle2RobotLink.srv
   OpenProblem.srv   
   OpenProblemStream.srv
   SetGoal.srv
   SetPlannerByName.srv
   SetRobControlsStream.srv
   CheckCollision.srv   
   ProblemOpened.srv
   SetInitObs.srv
   SetPlannerParameter.srv
   SetRobotsConfig.srv
   ClearSampleSet.srv    
   RemoveObstacle.srv
   SetInit.srv
   SetPlanner.srv
   Solve.srv
   CloseProblem.srv
   RemoveRobot.srv
   SetObsControls.srv
   SetPlannerStream.srv
   Connect.srv
   GetLastPlanComputationTime.srv
   GetNumEdges.srv
   GetNumVertices.srv
   #manipulation_node services
   OpenManipProblem.srv  
   ManipulationAction.srv
   SetWorldState.srv
   GetWorldState.srv
   GetBodyState.srv
   SetBodyState.srv
   SetManipQuery.srv

)

add_message_files(
   DIRECTORY msg
   FILES 
   fVector.msg
   dVector.msg
)

generate_messages(DEPENDENCIES roscpp std_msgs trajectory_msgs)


catkin_package(
  INCLUDE_DIRS         ${CMAKE_CURRENT_SOURCE_DIR}                   
  CATKIN_DEPENDS roscpp trajectory_msgs std_msgs
)

endif( KAUTHAM_ROS )


##################### ROS-catkin ####################################

# Building librarires

add_subdirectory ( src )

##############################################################################
# Documentation
# Tip from
# http://mementocodex.wordpress.com/2013/01/19/how-to-generate-code-documentation-with-doxygen-and-cmake-a-slightly-improved-approach/

if(KAUTHAM_DOCUMENTATION)
  find_package(Doxygen)
  if(NOT DOXYGEN_FOUND)
    message(FATAL_ERROR
      "Doxygen is needed to build the documentation.")
  endif()

  set( doxyfile_in          ${CMAKE_CURRENT_SOURCE_DIR}/doc/Doxyfile.in     )
  set( doxyfile             ${PROJECT_BINARY_DIR}/Doxyfile              )
  set( doxy_html_index_file ${CMAKE_CURRENT_BINARY_DIR}/html/index.html )
  set( doxy_output_root     ${CMAKE_CURRENT_BINARY_DIR}                 ) #  Pasted into Doxyfile.in
  set( doxy_input           ${PROJECT_SOURCE_DIR}/src                   ) #  Pasted into Doxyfile.in
#  set( doxy_extra_files     ${CMAKE_CURRENT_SOURCE_DIR}/mainpage.dox    ) #  Pasted into Doxyfile.in

  configure_file( ${doxyfile_in} ${doxyfile} @ONLY )

  add_custom_command( OUTPUT ${doxy_html_index_file}
                      COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
                      # The following should be ${doxyfile} only but it
                      # will break the dependency.
                      # The optimal solution would be creating a
                      # custom_command for ${doxyfile} generation
                      # but I still have to figure out how...
                      MAIN_DEPENDENCY ${doxyfile} ${doxyfile_in}
                      DEPENDS Kautham2 ${doxy_extra_files}
                      COMMENT "Generating HTML documentation")

  add_custom_target( doc DEPENDS ${doxy_html_index_file} )

  install( DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doxygen_documentation/html DESTINATION share/kautham/doc)
endif()
##############################################################################
#########################################################
# Copy Help file
install(DIRECTORY ${CMAKE_SOURCE_DIR} DESTINATION share/kautham2)
##install(DIRECTORY ${CMAKE_SOURCE_DIR}/demos DESTINATION share/kautham)





