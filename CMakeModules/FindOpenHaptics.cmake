# - Find OpenHaptics
# Find the native OPENHAPTICS headers and libraries.
#
#  OPENHAPTICS_INCLUDE_DIR -  where to find OpenHaptics.h, etc.
#  OPENHAPTICS_LIBRARIES    - List of libraries when using OpenHaptics.
#  OPENHAPTICS_FOUND        - True if OpenHaptics found.


# Look for the header file.
find_path(OPENHAPTICS_INCLUDE_DIR NAMES HL/hl.h HD/hd.h
                                  PATHS $ENV{3DTOUCH_BASE}/include
                                        "/Program Files/SensAble/3DTouch/include")
mark_as_advanced(OPENHAPTICS_INCLUDE_DIR)

find_path(OPENHAPTICS_INCLUDE_UTILITIES_DIR NAMES HDU/hdu.h
                                  PATHS $ENV{3DTOUCH_BASE}/utilities/include
                                        "/Program Files/SensAble/3DTouch/utilities/include")
mark_as_advanced(OPENHAPTICS_INCLUDE_UTILITIES_DIR)


# Look for the library.
find_library(HL_LIBRARY NAMES HL 
                        PATHS $ENV{3DTOUCH_BASE}/lib
							  $ENV{3DTOUCH_BASE}/lib/Win32
                              "/Program Files/SensAble/3DTouch/lib")
mark_as_advanced(HL_LIBRARY)

find_library(HD_LIBRARY NAMES HD
                        PATHS $ENV{3DTOUCH_BASE}/lib
							  $ENV{3DTOUCH_BASE}/lib/Win32
                              "/Program Files/SensAble/3DTouch/lib")
mark_as_advanced(HD_LIBRARY)

find_library(HDU_LIBRARY NAMES HDU
                         PATHS $ENV{3DTOUCH_BASE}/utilities/lib
						       $ENV{3DTOUCH_BASE}/utilities/lib/Win32/Release
                               "/Program Files/SensAble/3DTouch/utilities/lib")
mark_as_advanced(HDU_LIBRARY)

# Copy the results to the output variables.
if(OPENHAPTICS_INCLUDE_DIR AND OPENHAPTICS_INCLUDE_UTILITIES_DIR AND HD_LIBRARY AND HL_LIBRARY AND HDU_LIBRARY)
  set(OPENHAPTICS_FOUND 1)
  set(OPENHAPTICS_LIBRARIES ${HD_LIBRARY} ${HL_LIBRARY} ${HDU_LIBRARY})
  set(OPENHAPTICS_INCLUDE_DIR ${OPENHAPTICS_INCLUDE_DIR} ${OPENHAPTICS_INCLUDE_UTILITIES_DIR} )
else(OPENHAPTICS_INCLUDE_DIR AND OPENHAPTICS_INCLUDE_UTILITIES_DIR AND HD_LIBRARY AND HL_LIBRARY AND HDU_LIBRARY)
  set(OPENHAPTICS_FOUND 0)
  set(OPENHAPTICS_LIBRARIES)
  set(OPENHAPTICS_INCLUDE_DIR)
endif(OPENHAPTICS_INCLUDE_DIR AND OPENHAPTICS_INCLUDE_UTILITIES_DIR AND HD_LIBRARY AND HL_LIBRARY AND HDU_LIBRARY)

# Report the results.
if(NOT OPENHAPTICS_FOUND)
  set(OPENHAPTICS_DIR_MESSAGE
    "OPENHAPTICS was not found. Make sure to set OPENHAPTICS_LIBRARY and OPENHAPTICS_INCLUDE_DIR to the location of the library. If you do not have it you will not be able to use haptics devices from SensAble Technologies such as the Phantom.")
  if(OPENHAPTICS_FIND_REQUIRED)
    message(FATAL_ERROR "${OPENHAPTICS_DIR_MESSAGE}")
  elseif(NOT OPENHAPTICS_FIND_QUIETLY)
    message(STATUS "${OPENHAPTICS_DIR_MESSAGE}")
  endif(OPENHAPTICS_FIND_REQUIRED)
endif(NOT OPENHAPTICS_FOUND)
