cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
#rosbuild_add_executable(publiR1 src/publiR1.cpp)
#rosbuild_add_executable(suscriR1 src/suscriR1.cpp)
rosbuild_add_executable(remoteServer src/remoteserver.cpp src/connection.hpp src/connection.cpp )
rosbuild_add_executable(localClient src/localclient.cpp)
#rosbuild_add_executable(tf_Staubli_Server src/tf_staubli_server.cpp)
#rosbuild_add_executable(tf_Staubli_Client src/tf_staubli_client.cpp)
#rosbuild_add_executable(odo_haptic src/odo_haptic.cpp)
#rosbuild_add_executable(btn_haptic src/btn_haptic.cpp)
#rosbuild_add_executable(frc_haptic src/frc_haptic.cpp)
#rosbuild_add_executable(stt_haptic src/stt_haptic.cpp)

ADD_SUBDIRECTORY( CS8Soap )
SET(mt_DIR ${CMAKE_SOURCE_DIR}/mt)
FIND_PACKAGE( mt REQUIRED )
INCLUDE_DIRECTORIES( CS8Soap CS8Soap/src ${mt_DIR} )
#rosbuild_link_boost( publiR1 interprocess )
#rosbuild_link_boost( suscriR1 interprocess )

#TARGET_LINK_LIBRARIES( tf_Staubli_Server libsoapstaubli  )
TARGET_LINK_LIBRARIES( remoteServer cs8soap  )

