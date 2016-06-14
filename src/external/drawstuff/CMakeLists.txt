cmake_minimum_required(VERSION 2.8)
project(libdrawstuff)

# Include GNUInstallDirs to get canonical paths
include(GNUInstallDirs)

include_directories(${CMAKE_SOURCE_DIR}/include)
set(libdrawstuff_SOURCES src/drawstuff.cpp src/internal.h)

if(WIN32)
   set(libdrawstuff_SOURCES ${libdrawstuff_SOURCES} 
	src/windows.cpp src/resource.h src/resources.rc)
   set(libdrawstuff_LIBADD "-lwinmm -lgdi32")
   set(libdrawstuff_LDFLAGS "-no-undefined")
endif()
if(APPLE)
   set(libdrawstuff_SOURCES ${libdrawstuff_SOURCES} src/osx.cpp)
endif()
if(UNIX)
   set(libdrawstuff_SOURCES ${libdrawstuff_SOURCES} src/x11.cpp)
#   set(libdrawstuff_LIBADD "$(X11_LIBS)")
endif()

add_library(drawstuff SHARED ${libdrawstuff_SOURCES})
set_target_properties(drawstuff PROPERTIES 
	COMPILE_FLAGS -DDEFAULT_PATH_TO_TEXTURES='"${CMAKE_SOURCE_DIR}/textures/"')

#VERSION ${CCD_VERSION} SOVERSION ${CCD_SOVERSION})

#install(TARGETS ccd ccd_static ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
#   LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
#install(DIRECTORY src/ccd DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.h")
#install(FILES "${pkg_conf_file}" DESTINATION
#  ${CMAKE_INSTALL_LIBDIR}/pkgconfig COMPONENT pkgconfig)
