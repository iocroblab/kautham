
PROJECT(Kautham2)

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
#CMAKE_POLICY(SET CMP0003 OLD)



########### Options ####################
OPTION( KAUTHAM_GUI "Build the gui version" ON )
OPTION( KAUTHAM_CONSOLE "Build the console version" ON )
OPTION( KAUTHAM_OMPL "Use OMPL planners" ON )
OPTION( KAUTHAM_OMPL_LATEST "Use latest OMPL version" ON )
OPTION( KAUTHAM_IOC "Use IOC planners" ON )
OPTION( KAUTHAM_GUIBRO "Use Guibro planner (virtual bronchoscopies)" OFF)
OPTION( KAUTHAM_OPENDE "Use Open Dynamic Engine" ON)
OPTION( KAUTHAM_DOCUMENTATION "Create and install the HTML based API documentation (requires Doxygen)" ON)


########### Set flags ####################
MESSAGE( STATUS "flags: " ${KAUTHAM_GUI} " " ${KAUTHAM_CONSOLE} " "  ${KAUTHAM_OMPL}  " "  ${KAUTHAM_IOC} " "  ${KAUTHAM_GUIBRO} " " ${KAUTHAM_OPENDE})
IF( KAUTHAM_GUI )
  MESSAGE( STATUS "prepared to make KAUTHAM GUI" )
ENDIF( KAUTHAM_GUI )

IF( KAUTHAM_CONSOLE )
  MESSAGE( STATUS "prepared to make KAUTHAM CONSOLE" )
ENDIF( KAUTHAM_CONSOLE )

IF( KAUTHAM_OMPL )
  MESSAGE( STATUS "prepared to include OMPL" )
ENDIF( KAUTHAM_OMPL )

IF( KAUTHAM_IOC )
    MESSAGE( STATUS "Use IOC planners: setting -DKAUTHAM_USE_IOC flag")
    ADD_DEFINITIONS( -DKAUTHAM_USE_IOC )
ENDIF( KAUTHAM_IOC )

#IF( KAUTHAM_GUIBRO )
#    MESSAGE( STATUS "Use GUIBRO: setting -DKAUTHAM_USE_GUIBRO flag")
#    ADD_DEFINITIONS( -DKAUTHAM_USE_GUIBRO )
#ENDIF( KAUTHAM_GUIBRO )

IF( KAUTHAM_OPENDE )
  MESSAGE( STATUS "prepared to include ODE" )
ENDIF( KAUTHAM_OPENDE )

IF( KAUTHAM_DOCUMENTATION)
   message("prepared to and install the HTML based API documentation")
ENDIF(KAUTHAM_DOCUMENTATION)

########### Modules path ###############
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

##############################################################################
##############################################################################
# START General required packages

MESSAGE(STATUS "Configurating external packages ...")

##### pugixml ########
FIND_PACKAGE(PugiXML REQUIRED)
IF(PUGI_XML_FOUND)
    MESSAGE( STATUS "PugiXML FOUND, using it")
ELSE(PUGI_XML_FOUND)
    MESSAGE(STATUS "PugiXML not found, using internal copy")
    ADD_SUBDIRECTORY ( src/external/pugixml )
    SET(PUGI_XML_LIBRARIES libpugixml)
ENDIF(PUGI_XML_FOUND)


##############################################################################
##############################################################################
# START General required packages

#### mt package ##################
# utility functions for managing transforms
SET(mt_DIR ${CMAKE_SOURCE_DIR}/src/util/libmt)
MESSAGE (STATUS "Adding libmt directory ${mt_DIR}")


#### gdiam package ##################
# utility functions for finding minimum-volume bounding boxes
SET(gdiam_DIR ${CMAKE_SOURCE_DIR}/src/external/gdiam)
MESSAGE (STATUS "Adding gdiam directory")
ADD_SUBDIRECTORY ( src/external/gdiam )

#### pqp package ##################
# collision library
SET(pqp_DIR ${CMAKE_SOURCE_DIR}/src/external/pqp)
MESSAGE (STATUS "Adding PQP directory")
ADD_SUBDIRECTORY ( src/external/pqp )

#### libann package ##################
#   The Nearest Neighbor Library for Motion Planning

SET(DNN_DIR ${CMAKE_SOURCE_DIR}/src/external/libann)
MESSAGE (STATUS "Adding libann directory")
ADD_SUBDIRECTORY ( src/external/libann )

#### Boost package ###############
FIND_PACKAGE( Boost COMPONENTS  system serialization thread REQUIRED )
IF(Boost_FOUND)
  MESSAGE(STATUS "Boost library found at first attempt.")
ELSE(Boost_FOUND)
  SET( BOOST_ROOT $ENV{BOOSTDIR})
        MESSAGE(STATUS "Boost library NOT found at first attempt. Setting BOOST_ROOT to" $ENV{BOOSTDIR})
        FIND_PACKAGE( Boost COMPONENTS  system serialization thread REQUIRED )
        IF(Boost_FOUND)
                MESSAGE(STATUS "Boost library found in " ${BOOST_ROOT})
                INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
                LINK_DIRECTORIES( ${Boost_LIBRARY_DIRS} )
        ELSE(Boost_FOUND)
                MESSAGE(STATUS "Boost library not found. Boost-dependent headers will NOT be available")
        ENDIF(Boost_FOUND)
ENDIF(Boost_FOUND)

#### Coin3D package ###############
# graphics library
FIND_PACKAGE( Coin REQUIRED )
#INCLUDE_DIRECTORIES( ${COIN_INCLUDE_DIR} )

# END General required packages
##############################################################################

##############################################################################
##############################################################################
# START Required packages when using OMPL planners
IF( KAUTHAM_OMPL )
    #### OMPL package ###############
    find_package(ompl)
    IF( OMPL_FOUND )
        MESSAGE( STATUS "OMPL Package FOUND: setting -DKAUTHAM_USE_OMPL flag")
        MESSAGE( STATUS "OMPL lib is: " ${OMPL_LIBRARIES})
        MESSAGE( STATUS "OMPL include is: "${OMPL_INCLUDE_DIRS})
        ADD_DEFINITIONS( -DKAUTHAM_USE_OMPL )
        MESSAGE( STATUS "OMPL VERSION is: " ${OMPL_VERSION})

        IF(OMPL_EXTENSION_OPENDE)

        IF(KAUTHAM_OPENDE)
        find_package(OpenDE REQUIRED)
            IF(ODE_FOUND)
                MESSAGE( STATUS "OMPL Package FOUND: setting -DKAUTHAM_USE_ODE flag")
                ADD_DEFINITIONS( -DKAUTHAM_USE_ODE )
            ENDIF(ODE_FOUND)
        ENDIF(KAUTHAM_OPENDE)

        ENDIF(OMPL_EXTENSION_OPENDE)
    ELSE( OMPL_FOUND )
        MESSAGE( STATUS "Error: OMPL Package NOT FOUND. OMPL planers will not be abaliable")
    ENDIF( OMPL_FOUND )
ENDIF( KAUTHAM_OMPL )
# END Required packages when using OMPL planners
##############################################################################


##############################################################################
##############################################################################
# START Required packages when using IOC planners
IF( KAUTHAM_IOC )
  #### GSL package ###############
  # maths lobrary used in libproblem for the SAH hand inverse kinematics
  FIND_PACKAGE( GSL )
  IF( GSL_FOUND )
    MESSAGE( STATUS "GSL FOUND.")
     ADD_DEFINITIONS( -DNDEBUG )
     ADD_DEFINITIONS( -DKAUTHAM_USE_GSL )
  ELSE( GSL_FOUND )
    MESSAGE( STATUS "GSL NOT FOUND. Hand inverse kinematics will be disabled")
  ENDIF( GSL_FOUND )

  #### Armadillo package ###############
  #used for PCA
  FIND_PACKAGE( Armadillo )
  IF( ARMADILLO_FOUND )
    MESSAGE( STATUS "armadillo Package FOUND")
    ADD_DEFINITIONS( -DKAUTHAM_USE_ARMADILLO )
  ELSE( ARMADILLO_FOUND )
    MESSAGE( STATUS "armadillo Package NOT FOUND. Planners using PCA will be disabled.")
  ENDIF( ARMADILLO_FOUND )
ENDIF( KAUTHAM_IOC )
# END Required packages when using IOC planners
##############################################################################


##############################################################################
# START Required packages for GUI
  FIND_PACKAGE( Qt4 REQUIRED QtCore QtGui QtXml )
  IF( QT4_FOUND )
    include(${QT_USE_FILE})
    #ADD_DEFINITIONS( ${QT_DEFINITIONS} )
  ENDIF( QT4_FOUND )




##############################################################################

  FIND_PACKAGE( SoQt4 REQUIRED )
  IF( SOQT_LIBRARY_FOUND )
    ADD_DEFINITIONS( -DSOQT_DLL -DCOIN_DLL )
    #INCLUDE_DIRECTORIES( ${SOQT_INCLUDE_DIR} )
  ENDIF( SOQT_LIBRARY_FOUND )
# END Required packages for GUI
##############################################################################



##############################################################################
# Building librarires
  ADD_SUBDIRECTORY ( src )




##############################################################################
# Documentation
# Tip from
# http://mementocodex.wordpress.com/2013/01/19/how-to-generate-code-documentation-with-doxygen-and-cmake-a-slightly-improved-approach/

option(BUILD_DOCUMENTATION "Create and install the HTML based API
documentation (requires Doxygen)" ON)
IF(BUILD_DOCUMENTATION)
  FIND_PACKAGE(Doxygen)
  IF(NOT DOXYGEN_FOUND)
    MESSAGE(FATAL_ERROR
      "Doxygen is needed to build the documentation.")
  ENDIF()

  SET( doxyfile_in          ${CMAKE_CURRENT_SOURCE_DIR}/doc/Doxyfile.in     )
  SET( doxyfile             ${PROJECT_BINARY_DIR}/Doxyfile              )
  SET( doxy_html_index_file ${CMAKE_CURRENT_BINARY_DIR}/html/index.html )
  SET( doxy_output_root     ${CMAKE_CURRENT_BINARY_DIR}                 ) #  Pasted into Doxyfile.in
  SET( doxy_input           ${PROJECT_SOURCE_DIR}/src                   ) #  Pasted into Doxyfile.in
#  SET( doxy_extra_files     ${CMAKE_CURRENT_SOURCE_DIR}/mainpage.dox    ) #  Pasted into Doxyfile.in

  CONFIGURE_FILE( ${doxyfile_in} ${doxyfile} @ONLY )

  ADD_CUSTOM_COMMAND( OUTPUT ${doxy_html_index_file}
                      COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
                      # The following should be ${doxyfile} only but it
                      # will break the dependency.
                      # The optimal solution would be creating a
                      # custom_command for ${doxyfile} generation
                      # but I still have to figure out how...
                      MAIN_DEPENDENCY ${doxyfile} ${doxyfile_in}
                      DEPENDS Kautham2 ${doxy_extra_files}
                      COMMENT "Generating HTML documentation")

  ADD_CUSTOM_TARGET( doc DEPENDS ${doxy_html_index_file} )

  INSTALL( DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html DESTINATION share/doc)
ENDIF()
##############################################################################



##############################################################################
# Copy Help file

INSTALL(DIRECTORY ${CMAKE_SOURCE_DIR}/demos DESTINATION share/kautham)



