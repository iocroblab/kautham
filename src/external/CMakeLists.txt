##############################################################################
# START General required packages

message(STATUS "Configurating external packages ...")



##### pugixml ########
find_package( PugiXML )
if(PUGI_XML_FOUND)
#   message( STATUS "PugiXML FOUND, using it")
#   set(EXTRA_LIBS ${EXTRA_LIBS} ${PUGI_XML_LIBRARIES} PARENT_SCOPE  )
#   include_directories( ${PUGI_XML_INCLUDE_DIR})
else(PUGI_XML_FOUND)
#   message(STATUS "PugiXML not found, using internal copy")
   add_subdirectory ( pugixml )
#   include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/pugixml )
   #set(PUGI_XML_LIBRARIES libpugixml)
endif(PUGI_XML_FOUND)

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

