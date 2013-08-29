PROJECT(Kautham2)

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
CMAKE_POLICY(SET CMP0003 OLD)

MESSAGE(STATUS "build dir=" ${CMAKE_BINARY_DIR})
MESSAGE(STATUS "source dir=" ${CMAKE_CURRENT_SOURCE_DIR})
execute_process(COMMAND rm ${CMAKE_BINARY_DIR}/CMakeCache.txt)

########### Options ####################
OPTION( KAUTHAM_GUI "Build the gui version" ON )
OPTION( KAUTHAM_CONSOLE "Build the console version" OFF )
OPTION( KAUTHAM_OMPL "Use OMPL planners" OFF )
OPTION( KAUTHAM_IOC "Use IOC planners" ON )
OPTION( KAUTHAM_GUIBRO "Use Guibro planner (virtual bronchoscopies)" OFF )



########### Set flags ####################
MESSAGE( STATUS "flags: " ${KAUTHAM_GUI} " " ${KAUTHAM_CONSOLE} " "  ${KAUTHAM_OMPL}  " "  ${KAUTHAM_IOC} " "  ${KAUTHAM_GUIBRO} )
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

IF( KAUTHAM_GUIBRO )
    MESSAGE( STATUS "Use GUIBRO: setting -DKAUTHAM_USE_GUIBRO flag")
    ADD_DEFINITIONS( -DKAUTHAM_USE_GUIBRO )
ENDIF( KAUTHAM_GUIBRO )




########### Modules path ###############
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/modules)



##############################################################################
##############################################################################
# START General required packages

#### mt package ##################
# utility functions for managing transforms
SET(mt_DIR ${CMAKE_SOURCE_DIR}/libsutil/libmt)
INCLUDE_DIRECTORIES( libsutil/libmt )

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
INCLUDE_DIRECTORIES( ${COIN_INCLUDE_DIR} )

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
        ADD_DEFINITIONS( -DKAUTHAM_USE_OMPL )
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
# Building librarires
  INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR} )
  INCLUDE_DIRECTORIES( libsutil )
  INCLUDE_DIRECTORIES( libsexternal )
  ADD_SUBDIRECTORY ( libsutil/libkin )
  ADD_SUBDIRECTORY ( libsutil/libann )
  ADD_SUBDIRECTORY ( libsutil/libkthutil )
  ADD_SUBDIRECTORY ( libsutil/libpugixml )
  ADD_SUBDIRECTORY ( libsutil/libpqp )
  ADD_SUBDIRECTORY ( libproblem )
  ADD_SUBDIRECTORY ( libsampling )
  ADD_SUBDIRECTORY ( libplanner )




##############################################################################
# START building Kautham GUI application
IF( KAUTHAM_GUI )
  MESSAGE( STATUS "making KAUTHAM GUI" )

##############################################################################
# START Required packages for GUI
  FIND_PACKAGE( Qt4 REQUIRED )
  IF( QT4_FOUND )
    SET( QT_USE_QTMAIN TRUE )
    INCLUDE ( ${QT_USE_FILE} )
    INCLUDE_DIRECTORIES( ${QT_INCLUDE_DIR} )
    ADD_DEFINITIONS( ${QT_DEFINITIONS} )
  ENDIF( QT4_FOUND )

  FIND_PACKAGE( SoQt4 REQUIRED )
  IF( SOQT_LIBRARY_FOUND )
    ADD_DEFINITIONS( -DSOQT_DLL -DCOIN_DLL )
    INCLUDE_DIRECTORIES( ${SOQT_INCLUDE_DIR} )
  ENDIF( SOQT_LIBRARY_FOUND )
# END Required packages for GUI
##############################################################################

##############################################################################
# Building librarires
  ADD_SUBDIRECTORY ( libgui )
  IF( KAUTHAM_GUIBRO )
    MESSAGE( STATUS "Creating libguibro")
    INCLUDE_DIRECTORIES( libsexternal )
    ADD_SUBDIRECTORY ( libsexternal/libguibro )
    SET(KC_EXTRA_LIBS ${KC_EXTRA_LIBS}  libguibro)
    MESSAGE( STATUS "KC_EXTRA_LIBSs: " ${KC_EXTRA_LIBS})
  ENDIF( KAUTHAM_GUIBRO )



##############################################################################
# Creating executable
  FILE( GLOB RES_FILES *.rc )
  QT4_WRAP_CPP( QT_MOC_SRC application.h )
  
  SET( SRC_FILES_GUI application.cpp kauthamplanner.cpp )
  SET( HDR_FILES_GUI application.h )
  
  ADD_EXECUTABLE(Kautham2  ${SRC_FILES_GUI} ${HDR_FILES_GUI} ${QT_MOC_SRC} ${RES_FILES} )
  TARGET_LINK_LIBRARIES( Kautham2 ${KC_EXTRA_LIBS} libDNN libPQP libgui libsampling
                                                libproblem libkthutil libkin libpugixml)
ENDIF( KAUTHAM_GUI )

# END building Kautham GUI application
##############################################################################



##############################################################################
##############################################################################
# START building Kautham Console application
IF( KAUTHAM_CONSOLE )

  MESSAGE( STATUS "making KAUTHAM CONSOLE" )

  #MPI Library used to turn the KauthamConsole parallel process.
  INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}  ${mt_DIR} ${COIN_INCLUDE_DIR} ${CMAKE_SOURCE_DIR}/libgui )
  #FIND_PACKAGE( MPI )
  IF( MPI_FOUND )
    INCLUDE_DIRECTORIES( ${MPI_INCLUDE_PATH} )
    ADD_DEFINITIONS( -DKAUTHAM_USE_MPI )
    MESSAGE( STATUS "MPI library found at" ${MPI_INCLUDE_PATH} )
    SET(KC_EXTRA_LIBS ${KC_EXTRA_LIBS} ${MPI_LIBRARIES} )
    MESSAGE( STATUS "MPI_LIBRARIES = " ${MPI_LIBRARIES} )
  ELSE( MPI_FOUND )
    MESSAGE( STATUS "MPI library not found" )
  ENDIF( MPI_FOUND )

  SET( SRC_FILES kauthamplanner_console.cpp libutil/kauthamobject.cpp )
  
    # Adding console bath mode executable
  ADD_EXECUTABLE( KauthamConsole ${SRC_FILES} ${HDR_FILES} ${RES_FILES} )
  TARGET_LINK_LIBRARIES( KauthamConsole libDNN libPQP libproblem libkthutil libkin libpugixml ${KC_EXTRA_LIBS} )
  
  MESSAGE( STATUS "KC_EXTRA_LIBS = " ${KC_EXTRA_LIBS} )
ENDIF( KAUTHAM_CONSOLE )

# END building Kautham Console application
##############################################################################




