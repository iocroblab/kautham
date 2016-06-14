#     Copyright 2016 Institute of Industrial and Control Engineering (IOC)
#                  Universitat Politecnica de Catalunya
#                  BarcelonaTech
#     All Rights Reserved.
# 
#     This program is free software; you can redistribute it and/or modify
#     it under the terms of the GNU General Public License as published by
#     the Free Software Foundation; either version 2 of the License, or
#     (at your option) any later version.
# 
#     This program is distributed in the hope that it will be useful,
#     but WITHOUT ANY WARRANTY; without even the implied warranty of
#     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#     GNU General Public License for more details.
# 
#     You should have received a copy of the GNU General Public License
#     along with this program; if not, write to the
#     Free Software Foundation, Inc.,
#     59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
# 
#    Author: Leopold Palomo-Avellaneda 2014-2016



project(kautham)

cmake_minimum_required(VERSION 2.4.6)

set( KAUTHAM_VERSION 3.0.0 )
STRING( REGEX MATCHALL "[0-9]+" KAUTHAM_VERSIONS ${KAUTHAM_VERSION} ) 
LIST( GET KAUTHAM_VERSIONS 0 KAUTHAM_VERSION_MAJOR)
LIST( GET KAUTHAM_VERSIONS 1 KAUTHAM_VERSION_MINOR)
LIST( GET KAUTHAM_VERSIONS 2 KAUTHAM_VERSION_PATCH)

message( "Kautham version ${VERSION} (${KAUTHAM_VERSION_MAJOR}.${KAUTHAM_VERSION_MINOR}.${KAUTHAM_VERSION_PATCH})" )

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
option( KAUTHAM_ROS "Build the ROS version" OFF )
option( KAUTHAM_DOCUMENTATION "Create and install the HTML based API documentation (requires Doxygen)" OFF)

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

########### Modules path and system options ###############
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)
include(GNUInstallDirs)
##############################################################################

##################### ROS-catkin ####################################


if( KAUTHAM_ROS )
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  trajectory_msgs
  message_generation
)
include_directories(${catkin_INCLUDE_DIRS})

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
   SolveManipQuery.srv

)

add_message_files(
   DIRECTORY msg
   FILES 
   fVector.msg
   dVector.msg
   odeState.msg
)

generate_messages(DEPENDENCIES roscpp std_msgs trajectory_msgs)


catkin_package(
  #INCLUDE_DIRS  ${CMAKE_SOURCE_DIR}/src
  INCLUDE_DIRS  ${CMAKE_SOURCE_DIR}
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
                      DEPENDS kautham ${doxy_extra_files}
                      COMMENT "Generating HTML documentation")

  add_custom_target( doc DEPENDS ${doxy_html_index_file} )

  install( DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doxygen_documentation/html DESTINATION share/kautham/doc)
endif()
##############################################################################
#########################################################
# Copy Help file
#install(DIRECTORY ${CMAKE_SOURCE_DIR} DESTINATION share/kautham)
install(DIRECTORY ${CMAKE_SOURCE_DIR}/demos DESTINATION share/kautham)





