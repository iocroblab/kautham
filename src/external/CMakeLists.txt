##############################################################################
# START General required packages

message(STATUS "Configurating external packages ...")

#### gdiam package ##################
# utility functions for finding minimum-volume bounding boxes
set(gdiam_DIR gdiam)
message (STATUS "Adding gdiam directory")
add_subdirectory ( gdiam )

#### pqp package ##################
# collision library
set(pqp_DIR pqp)
message (STATUS "Adding PQP directory")
add_subdirectory ( pqp )

set(KAUTHAM_SRC ${KAUTHAM_SRC} PARENT_SCOPE )

