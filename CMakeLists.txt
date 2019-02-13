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
#    Author: Leopold Palomo-Avellaneda 2014-2017



project(kautham)

cmake_minimum_required(VERSION 2.4.6)

add_definitions(-std=c++11 -Wall -Wextra -Wpedantic )#-Werror)

set( KAUTHAM_VERSION 3.2.4 )
STRING( REGEX MATCHALL "[0-9]+" KAUTHAM_VERSIONS ${KAUTHAM_VERSION} ) 
LIST( GET KAUTHAM_VERSIONS 0 KAUTHAM_VERSION_MAJOR)
LIST( GET KAUTHAM_VERSIONS 1 KAUTHAM_VERSION_MINOR)
LIST( GET KAUTHAM_VERSIONS 2 KAUTHAM_VERSION_PATCH)
# # configure a header file to pass some of the CMake settings to the source code
#  configure_file (
#    "${PROJECT_SOURCE_DIR}/kauthamConfig.h.in"
#   "${PROJECT_SOURCE_DIR}/include/kautham/kauthamConfig.h"
#   )

message( "Kautham version ${VERSION} (${KAUTHAM_VERSION_MAJOR}.${KAUTHAM_VERSION_MINOR}.${KAUTHAM_VERSION_PATCH})" )

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)


########### Options ####################
option( KAUTHAM_OMPL "Use OMPL planners" ON )
option( KAUTHAM_IOC "Use IOC planners" ON )
option( KAUTHAM_OPENDE "Use Open Dynamic Engine" ON)
option( KAUTHAM_GUI "Build the gui version" ON )
option( KAUTHAM_CONSOLE "Build the console version" ON )
option( KAUTHAM_ROS "Build the ROS version" OFF )
option( KAUTHAM_DOCUMENTATION_API "Create and install the HTML based API documentation (requires Doxygen)" ON)
option( KAUTHAM_DOCUMENTATION_WEB "Create WEB Documentation (requires Sphinx)" ON)

########### Set flags ####################
message( STATUS "Using this flags ........")
message( STATUS "Building the GUI version: ${KAUTHAM_GUI}")
message( STATUS "Building the console version: ${KAUTHAM_CONSOLE}")
message( STATUS "Building the ROS version: ${KAUTHAM_ROS}")
message( STATUS "Using OMPL planners: ${KAUTHAM_OMPL}")
message( STATUS "Using IOC planners: ${KAUTHAM_IOC}")
message( STATUS "Using Open Dynamic Engine: ${KAUTHAM_OPENDE}")
message( STATUS "Creating the HTML based API documentation: ${KAUTHAM_DOCUMENTATION}")

########### Modules path and system options ###############
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

##############################################################################

##################### ROS-catkin ####################################


if( KAUTHAM_ROS )

    find_package(catkin REQUIRED COMPONENTS
      roscpp
      std_msgs
      trajectory_msgs
      message_generation
    )
    include_directories(${catkin_INCLUDE_DIRS} include)

    add_service_files(
       DIRECTORY apps/ros/srv
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
       ObsPos.srv
       FindIK.srv
    )

    add_message_files(
       DIRECTORY apps/ros/msg
       FILES
       fVector.msg
    )

    generate_messages(DEPENDENCIES roscpp std_msgs trajectory_msgs)

    catkin_package(
      INCLUDE_DIRS include
      LIBRARIES kautham
      CATKIN_DEPENDS roscpp trajectory_msgs std_msgs
    )

endif( KAUTHAM_ROS )


##################### Qt-Creator ####################################

FILE(GLOB_RECURSE LibFiles include/*.h src/external/*.h src/external/gdiam/*.h src/external/pqp/*.h src/external/pugixml/*.h)
add_custom_target(headers SOURCES ${LibFiles})


#### Boost package ###############
find_package( Boost COMPONENTS  system serialization thread REQUIRED )
if(Boost_FOUND)
  message(STATUS "Boost library found at first attempt.")
else(Boost_FOUND)
  set( BOOST_ROOT $ENV{BOOSTDIR})
        message(STATUS "Boost library NOT found at first attempt. Setting BOOST_ROOT to" $ENV{BOOSTDIR})
        find_package( Boost COMPONENTS  system serialization thread REQUIRED )
        if(Boost_FOUND)
                message(STATUS "Boost library found in " ${BOOST_ROOT})
                include_directories(${Boost_INCLUDE_DIRS})
                set(EXTRA_LIBS ${EXTRA_LIBS} ${Boost_LIBRARIES})
                link_directories( ${Boost_LIBRARY_DIRS} )
        else(Boost_FOUND)
                message(STATUS "Boost library not found. Boost-dependent headers will NOT be available")
        endif(Boost_FOUND)
endif(Boost_FOUND)

#### Coin3D package ###############
# graphics library
find_package( Coin REQUIRED )
if( COIN_LIBRARY_FOUND )
   message( STATUS "Using Coin library")
   add_definitions( -DCOIN_DLL )
   include_directories( ${COIN_INCLUDE_DIR} )
    set(EXTRA_LIBS ${EXTRA_LIBS} ${COIN_LIBRARY})
else( COIN_LIBRARY_FOUND )
   message(SEND_ERROR "Kautham needs Coin library")
endif(COIN_LIBRARY_FOUND )

#### MT ###############
set(mt_DIR include/kautham/util/libmt)
message (STATUS "Adding libmt directory ${mt_DIR}")
include_directories( ${mt_DIR} )

### Eigen3 ############
find_package(Eigen3 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIR})

##################### ROS-catkin ####################################

include_directories( include )
include_directories( ${CMAKE_BINARY_DIR}/include/)

include_directories( src )  # for external


# Building librarires
add_subdirectory ( include )
add_subdirectory ( src )
add_subdirectory ( apps )
add_subdirectory ( doc )

#########################################################
# Copy Help file
# install(DIRECTORY include/kautham DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY demos DESTINATION share/kautham)
