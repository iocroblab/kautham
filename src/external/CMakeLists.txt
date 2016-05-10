##############################################################################
# START General required packages

message(STATUS "Configurating external packages ...")

#### gdiam package ##################
# utility functions for finding minimum-volume bounding boxes
set(gdiam_DIR ${CMAKE_SOURCE_DIR}/src/external/gdiam)
message (STATUS "Adding gdiam directory")
add_subdirectory ( gdiam )

#### pqp package ##################
# collision library
set(pqp_DIR ${CMAKE_SOURCE_DIR}/src/external/pqp)
message (STATUS "Adding PQP directory")
add_subdirectory ( pqp )

#### libann package ##################
#   The Nearest Neighbor Library for Motion Planning

set(DNN_DIR ${CMAKE_SOURCE_DIR}/src/external/libann)
message (STATUS "Adding libann directory")
add_subdirectory ( libann )

set(KAUTHAM_SRC ${KAUTHAM_SRC} PARENT_SCOPE )

