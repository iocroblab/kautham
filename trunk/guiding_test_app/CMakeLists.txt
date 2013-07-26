
PROJECT(guiding)

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
CMAKE_POLICY(SET CMP0003 OLD)

# Setup mt library
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/../modules )
SET(mt_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../libmt)

FIND_PACKAGE( mt REQUIRED )
FIND_PACKAGE( OpenHaptics REQUIRED)

##############################################################################
# Required common packages
##############################################################################


IF( WIN32 )
  ADD_DEFINITIONS( -D_CRT_SECURE_NO_DEPRECATE )
  ADD_DEFINITIONS( -D_WIN32_WINNT=0x0501 )
ENDIF( WIN32 )

#SET( PUGI_DIR G:/pugixml/src  )
SET( PUGI_DIR $ENV{PUGIDIR} )
SET( PUGI_INCLUDE_DIR ${PUGI_DIR} )
INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR} 
                     ${mt_DIR}
                     ${OPENHAPTICS_INCLUDE_DIR} 
                     ${PUGI_INCLUDE_DIR} )

FILE( GLOB SRC_FILES *.c* )
FILE(GLOB PUGI_SRC_FILES ${PUGI_INCLUDE_DIR}/*.c* )
FILE(GLOB PUGI_HDR_FILES ${PUGI_INCLUDE_DIR}/*.h*)

ADD_EXECUTABLE(guiding_test  ${SRC_FILES} ${PUGI_SRC_FILES} ${PUGI_HDR_FILES} )
TARGET_LINK_LIBRARIES( guiding_test ${OPENHAPTICS_LIBRARIES}  )





