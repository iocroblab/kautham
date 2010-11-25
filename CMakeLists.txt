
PROJECT(Kautham2)

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
CMAKE_POLICY(SET CMP0003 OLD)

# Setup mt library
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/modules )
SET(mt_DIR ${CMAKE_SOURCE_DIR}/libmt)

FIND_PACKAGE( mt REQUIRED )

##############################################################################
# Required common packages
##############################################################################

FIND_PACKAGE( Boost)
IF(Boost_FOUND)
  MESSAGE(STATUS "Boost library found at first attempt.")
ELSE(Boost_FOUND)
  SET( BOOST_ROOT $ENV{BOOSTDIR})
	FIND_PACKAGE( Boost REQUIRED  system serialization thread)
	IF(Boost_FOUND)
		MESSAGE(STATUS "Boost library found in " ${BOOST_ROOT})
        #INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
        LINK_DIRECTORIES(${Boost_LIBRARY_DIRS} )
	ELSE(Boost_FOUND)
		MESSAGE(STATUS "Boost library not found. Boost-dependent headers will NOT be available")
	ENDIF(Boost_FOUND)
ENDIF(Boost_FOUND)

FIND_PACKAGE( Coin REQUIRED )


IF( WIN32 )
  ADD_DEFINITIONS( -D_CRT_SECURE_NO_DEPRECATE )
  ADD_DEFINITIONS( -D_WIN32_WINNT=0x0501 )
ENDIF( WIN32 )

INCLUDE_DIRECTORIES( libutil )

ADD_SUBDIRECTORY ( libann )
ADD_SUBDIRECTORY ( libutil )
ADD_SUBDIRECTORY ( libpqp )
ADD_SUBDIRECTORY ( libproblem )
ADD_SUBDIRECTORY ( libsampling )
ADD_SUBDIRECTORY ( libplanner )


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
                     ${CMAKE_SOURCE_DIR}/libmt  )

ADD_SUBDIRECTORY ( libgui )

FILE( GLOB RES_FILES *.rc )
QT4_WRAP_CPP( QT_MOC_SRC application.h )

SET( SRC_FILES_GUI application.cpp kauthamplanner.cpp )
SET( HDR_FILES_GUI application.h )

ADD_EXECUTABLE(Kautham2  ${SRC_FILES_GUI} ${HDR_FILES_GUI} ${QT_MOC_SRC} ${RES_FILES} )
IF(WIN32) 
  SET_TARGET_PROPERTIES( Kautham2 PROPERTIES WIN32_EXECUTABLE "True" )
ENDIF(WIN32)

TARGET_LINK_LIBRARIES( Kautham2 libDNN libPQP libgui libsampling libproblem libkthutil )
