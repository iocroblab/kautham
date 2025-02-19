cmake_minimum_required(VERSION 3.5)
project(libdrawstuff)

# Include GNUInstallDirs to get canonical paths
include(GNUInstallDirs)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
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
endif()

add_library(drawstuff ${libdrawstuff_SOURCES})
set_target_properties(drawstuff PROPERTIES 
	COMPILE_FLAGS -DDEFAULT_PATH_TO_TEXTURES='"${CMAKE_SOURCE_DIR}/textures/"')

