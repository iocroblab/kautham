PROJECT(Kautham2)

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
CMAKE_POLICY(SET CMP0003 OLD)

# Setup mt library
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/modules
                      ${CMAKE_CURRENT_SOURCE_DIR}/libutil/armadillo/build_aux/cmake/Modules )
SET(mt_DIR ${CMAKE_SOURCE_DIR}/libmt)

FIND_PACKAGE( mt REQUIRED )

##############################################################################
# Required common packages
##############################################################################

FIND_PACKAGE( Boost COMPONENTS  system serialization thread REQUIRED )
IF(Boost_FOUND)
  MESSAGE(STATUS "Boost library found at first attempt.")
ELSE(Boost_FOUND)
  SET( BOOST_ROOT $ENV{BOOSTDIR})
	MESSAGE(STATUS "Boost library NOT found at first attempt. Setting BOOST_ROOT to" $ENV{BOOSTDIR})
	FIND_PACKAGE( Boost COMPONENTS  system serialization thread REQUIRED )
	IF(Boost_FOUND)
		MESSAGE(STATUS "Boost library found in " ${BOOST_ROOT})
        #INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
        LINK_DIRECTORIES(${Boost_LIBRARY_DIRS} )
	ELSE(Boost_FOUND)
		MESSAGE(STATUS "Boost library not found. Boost-dependent headers will NOT be available")
	ENDIF(Boost_FOUND)
ENDIF(Boost_FOUND)

FIND_PACKAGE( Coin REQUIRED )

FIND_PACKAGE( GSL )

FIND_PACKAGE( Armadillo )
IF( Armadillo_FOUND )
  MESSAGE( STATUS "armadillo Package FOUND")
  ADD_DEFINITIONS( -DKAUTHAM_USE_ARMADILLO )
ELSE( Armadillo_FOUND )
  IF( WIN32 )
    IF( "$ENV{ARMADILLO_DIR}" STREQUAL "" )
      MESSAGE( STATUS "armadillo Package NOT FOUND 1")
    ELSE()
      SET(Armadillo_DIR $ENV{ARMADILLO_DIR} )
      SET(Armadillo_INCLUDE_DIR ${Armadillo_DIR}/include )
      SET(Armadillo_LIBRARY ${Armadillo_DIR}/lib_win32/blas_win32_MT.lib 
                            ${Armadillo_DIR}/lib_win32/lapack_win32_MT.lib )
    
      MESSAGE( STATUS "Armadillo_INCLUDE_DIR: " ${Armadillo_INCLUDE_DIR} )
      ADD_DEFINITIONS( -DKAUTHAM_USE_ARMADILLO )
    ENDIF(  )
  ELSE()
    MESSAGE( STATUS "armadillo Package NOT FOUND 2")
  ENDIF( WIN32 )
ENDIF( Armadillo_FOUND )

IF( WIN32 )
  ADD_DEFINITIONS( -D_CRT_SECURE_NO_DEPRECATE )
  ADD_DEFINITIONS( -D_WIN32_WINNT=0x0501 )
ENDIF( WIN32 )

OPTION( KAUTHAM_GUI "Build the gui version" ON )
OPTION( USE_HAPTIC "Build with OpenHaptics" OFF )
OPTION( KAUTHAM_CONSOLE "Build the console version" OFF )

INCLUDE_DIRECTORIES( libutil )

ADD_SUBDIRECTORY ( libann )
ADD_SUBDIRECTORY ( libutil )
ADD_SUBDIRECTORY ( libpqp )
ADD_SUBDIRECTORY ( libproblem )
ADD_SUBDIRECTORY ( libsampling )
ADD_SUBDIRECTORY ( libplanner )

IF( KAUTHAM_GUI )
  FIND_PACKAGE( Qt4 REQUIRED )
  IF( QT4_FOUND )
    SET( QT_USE_QTMAIN TRUE )
    INCLUDE ( ${QT_USE_FILE} )
    ADD_DEFINITIONS( ${QT_DEFINITIONS} )
  ENDIF( QT4_FOUND )

  FIND_PACKAGE( SoQt4 REQUIRED )
  IF( SOQT_LIBRARY_FOUND )
    ADD_DEFINITIONS( -DSOQT_DLL -DCOIN_DLL )
    INCLUDE_DIRECTORIES( ${SOQT_INCLUDE_DIR} )
  ENDIF( SOQT_LIBRARY_FOUND )

  INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR} 
                       ${CMAKE_SOURCE_DIR}/libmt  
                       ${Armadillo_INCLUDE_DIR}
                       ${Boost_INCLUDE_DIRS})
                       
  LINK_DIRECTORIES(${Boost_LIBRARY_DIRS} )
                       
  SET(KC_EXTRA_LIBS ${Armadillo_LIBRARY} )
  
  ADD_SUBDIRECTORY ( libcomm )
  ADD_SUBDIRECTORY ( libdevice )
  ADD_SUBDIRECTORY ( libgui )
  ADD_SUBDIRECTORY ( libguiding )
    
  FILE( GLOB RES_FILES *.rc )
  QT4_WRAP_CPP( QT_MOC_SRC application.h )
  
  SET( SRC_FILES_GUI application.cpp kauthamplanner.cpp )
  SET( HDR_FILES_GUI application.h )
  
  ADD_EXECUTABLE(Kautham2  ${SRC_FILES_GUI} ${HDR_FILES_GUI} ${QT_MOC_SRC} ${RES_FILES} )
  IF(WIN32) 
    SET_TARGET_PROPERTIES( Kautham2 PROPERTIES WIN32_EXECUTABLE "True" )
  ENDIF(WIN32)

  TARGET_LINK_LIBRARIES( Kautham2 libDNN libPQP libdevice libgui libsampling 
						libproblem libioccomm libkthutil ${KC_EXTRA_LIBS} )
ENDIF( KAUTHAM_GUI )


IF( KAUTHAM_CONSOLE )
 # MPI Library used to turn the KauthamConsole parallel process.
  INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}  ${mt_DIR} ${COIN_INCLUDE_DIR} ${CMAKE_SOURCE_DIR}/libgui )
  FIND_PACKAGE( MPI )
  IF( MPI_FOUND )
    INCLUDE_DIRECTORIES( ${MPI_INCLUDE_PATH} )
    ADD_DEFINITIONS( -DKAUTHAM_USE_MPI )
    MESSAGE( STATUS "MPI library found at" ${MPI_INCLUDE_PATH} )
    SET(KC_EXTRA_LIBS ${KC_EXTRA_LIBS} ${MPI_LIBRARIES} )
  ELSE( MPI_FOUND )
    MESSAGE( STATUS "MPI library didn't found" )
  ENDIF( MPI_FOUND )

  SET( SRC_FILES kauthamplanner_console.cpp libgui/kauthamobject.cpp )
  
    # Adding console bath mode executable
  ADD_EXECUTABLE( KauthamConsole ${SRC_FILES} ${HDR_FILES} ${RES_FILES} )
  TARGET_LINK_LIBRARIES( KauthamConsole libDNN libPQP libproblem libkthutil ${KC_EXTRA_LIBS} )
ENDIF( KAUTHAM_CONSOLE )




