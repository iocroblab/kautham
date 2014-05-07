# - Find OpenHaptics
# Find the native OPENHAPTICS headers and libraries.
#
#  OPENHAPTICS_INCLUDE_DIR -  where to find OpenHaptics.h, etc.
#  OPENHAPTICS_LIBRARIES    - List of libraries when using OpenHaptics.
#  OPENHAPTICS_FOUND        - True if OpenHaptics found.


# Look for the header file.
FIND_PATH(OPENHAPTICS_INCLUDE_DIR NAMES HL/hl.h HD/hd.h
                                  PATHS $ENV{3DTOUCH_BASE}/include
                                        "/Program Files/SensAble/3DTouch/include")
MARK_AS_ADVANCED(OPENHAPTICS_INCLUDE_DIR)

FIND_PATH(OPENHAPTICS_INCLUDE_UTILITIES_DIR NAMES HDU/hdu.h
                                  PATHS $ENV{3DTOUCH_BASE}/utilities/include
                                        "/Program Files/SensAble/3DTouch/utilities/include")
MARK_AS_ADVANCED(OPENHAPTICS_INCLUDE_UTILITIES_DIR)


# Look for the library.
FIND_LIBRARY(HL_LIBRARY NAMES HL 
                        PATHS $ENV{3DTOUCH_BASE}/lib
							  $ENV{3DTOUCH_BASE}/lib/Win32
                              "/Program Files/SensAble/3DTouch/lib")
MARK_AS_ADVANCED(HL_LIBRARY)

FIND_LIBRARY(HD_LIBRARY NAMES HD
                        PATHS $ENV{3DTOUCH_BASE}/lib
							  $ENV{3DTOUCH_BASE}/lib/Win32
                              "/Program Files/SensAble/3DTouch/lib")
MARK_AS_ADVANCED(HD_LIBRARY)

FIND_LIBRARY(HDU_LIBRARY NAMES HDU
                         PATHS $ENV{3DTOUCH_BASE}/utilities/lib
						       $ENV{3DTOUCH_BASE}/utilities/lib/Win32/Release
                               "/Program Files/SensAble/3DTouch/utilities/lib")
MARK_AS_ADVANCED(HDU_LIBRARY)

# Copy the results to the output variables.
IF(OPENHAPTICS_INCLUDE_DIR AND OPENHAPTICS_INCLUDE_UTILITIES_DIR AND HD_LIBRARY AND HL_LIBRARY AND HDU_LIBRARY)
  SET(OPENHAPTICS_FOUND 1)
  SET(OPENHAPTICS_LIBRARIES ${HD_LIBRARY} ${HL_LIBRARY} ${HDU_LIBRARY})
  SET(OPENHAPTICS_INCLUDE_DIR ${OPENHAPTICS_INCLUDE_DIR} ${OPENHAPTICS_INCLUDE_UTILITIES_DIR} )
ELSE(OPENHAPTICS_INCLUDE_DIR AND OPENHAPTICS_INCLUDE_UTILITIES_DIR AND HD_LIBRARY AND HL_LIBRARY AND HDU_LIBRARY)
  SET(OPENHAPTICS_FOUND 0)
  SET(OPENHAPTICS_LIBRARIES)
  SET(OPENHAPTICS_INCLUDE_DIR)
ENDIF(OPENHAPTICS_INCLUDE_DIR AND OPENHAPTICS_INCLUDE_UTILITIES_DIR AND HD_LIBRARY AND HL_LIBRARY AND HDU_LIBRARY)

# Report the results.
IF(NOT OPENHAPTICS_FOUND)
  SET(OPENHAPTICS_DIR_MESSAGE
    "OPENHAPTICS was not found. Make sure to set OPENHAPTICS_LIBRARY and OPENHAPTICS_INCLUDE_DIR to the location of the library. If you do not have it you will not be able to use haptics devices from SensAble Technologies such as the Phantom.")
  IF(OPENHAPTICS_FIND_REQUIRED)
    MESSAGE(FATAL_ERROR "${OPENHAPTICS_DIR_MESSAGE}")
  ELSEIF(NOT OPENHAPTICS_FIND_QUIETLY)
    MESSAGE(STATUS "${OPENHAPTICS_DIR_MESSAGE}")
  ENDIF(OPENHAPTICS_FIND_REQUIRED)
ENDIF(NOT OPENHAPTICS_FOUND)
