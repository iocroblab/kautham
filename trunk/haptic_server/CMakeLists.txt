PROJECT(HapticServer)
 cmake_minimum_required(VERSION 2.6)
 cmake_policy(SET CMP0003 OLD)
 
# Setup mt library
SET(HARD_LAB ${CMAKE_SOURCE_DIR}/../hardlab)
set(mt_DIR ../libmt)
find_package(mt REQUIRED)
#include(mt_USE_PACKAGE)


# Setup OpenHaptic Library
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/../modules )

FIND_PACKAGE( OpenHaptics REQUIRED)

# Find boost library
 FIND_PACKAGE( Boost )
 IF(Boost_FOUND)
  message(STATUS "Boost library found at first attempt.")
ELSE(Boost_FOUND)
  SET( BOOST_ROOT $ENV{BOOSTDIR})
	FIND_PACKAGE( Boost REQUIRED)
	IF(Boost_FOUND)
		message(STATUS "Boost library found in " ${BOOST_ROOT})
	ELSE(Boost_FOUND)
		message(STATUS "Boost library not found. Boost-dependent headers will NOT be available")
	ENDIF(Boost_FOUND)
ENDIF(Boost_FOUND)

IF(Boost_FOUND)
	INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIRS} ${HARD_LAB})
	set(Boost_USE_MULTITHREADED ON)
	ADD_DEFINITIONS( -D_CRT_SECURE_NO_DEPRECATE )
	
	IF( WIN32 )
		ADD_DEFINITIONS( -D_WIN32_WINNT )
		ADD_DEFINITIONS( -D_WIN32_WINNT=0x0501 )
	ENDIF( WIN32 )
	
	INCLUDE_DIRECTORIES(${mt_DIR} ${OPENHAPTICS_INCLUDE_DIR} ${OPENHAPTICS_INCLUDE_UTILITIES_DIR} libcomm )

	LINK_DIRECTORIES(${Boost_LIBRARY_DIRS} ${IOC_COMM} ${OPENHAPTICS_LIBRARIES} ${CMAKE_SOURCE_DIR})
	
	ADD_SUBDIRECTORY( libcomm )
	 
	ADD_EXECUTABLE( haptic_server hapticserver.cpp ${HARD_LAB}/haptic/haptic.h 
					${HARD_LAB}/haptic/haptic.cpp )
	ADD_EXECUTABLE( haptic_client hapticclient.cpp )        
	
	IF ( WIN32 )
		TARGET_LINK_LIBRARIES( haptic_server libioccomm ${OPENHAPTICS_LIBRARIES} )
		TARGET_LINK_LIBRARIES( haptic_client libioccomm )
	ELSE ( WIN32 )
		TARGET_LINK_LIBRARIES( haptic_server comm boost_system boost_serialization boost_thread ${OPENHAPTICS_LIBRARIES} )
		TARGET_LINK_LIBRARIES( haptic_client comm boost_system boost_serialization boost_thread )
	ENDIF( WIN32 )
ELSE(Boost_FOUND)
   message(STATUS "Boost ASIO library not found.")
ENDIF(Boost_FOUND)
