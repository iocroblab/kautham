#### Adding subdirectories

include_directories( ${CMAKE_SOURCE_DIR}/src)
set(mt_DIR ${CMAKE_SOURCE_DIR}/src/util/libmt)
message (STATUS "Adding libmt directory ${mt_DIR}")
include_directories( ${mt_DIR} )

set ( kin_DIR ${CMAKE_CURRENT_SOURCE_DIR}/util/libkin )

set ( kthutil_DIR ${CMAKE_CURRENT_SOURCE_DIR}/util/kthutil )

set ( mt_DIR ${CMAKE_CURRENT_SOURCE_DIR}/util/libmt )

add_subdirectory( util )


INCLUDE_DIRECTORIES( include ${CMAKE_CURRENT_SOURCE_DIR} 
                       ${kthutil_DIR}
                       ${mt_DIR}
                       ${kin_DIR}                   
                       ${Boost_INCLUDE_DIRS} )

##### pugixml ########
find_package( PugiXML )
if(PUGI_XML_FOUND)
   message( STATUS "PugiXML FOUND, using it")
   set(EXTRA_LIBS ${EXTRA_LIBS} ${PUGI_XML_LIBRARIES} )
   include_directories( ${PUGI_XML_INCLUDE_DIR})
else(PUGI_XML_FOUND)
   message(STATUS "PugiXML not found, using internal copy")
#   add_subdirectory ( pugixml )
   include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/external/pugixml )
   #set(PUGI_XML_LIBRARIES libpugixml)
endif(PUGI_XML_FOUND)


add_subdirectory ( external )
#add_subdirectory( util )

#### Boost package ###############
find_package( Boost COMPONENTS  system serialization thread REQUIRED )
if(Boost_FOUND)
  message(STATUS "Boost library found at first attempt.")
else(Boost_FOUND)
  set( BOOST_ROOT $ENV{BOOSTDIR})
        message(STATUS "Boost library NOT found at first attempt. Setting BOOST_ROOT to" $ENV{BOOSTDIR})
        find_package( Boost COMPONENTS  system serialization thread REQUIRED )
        if(Boost_FOUND)
                message(STATUS "Boost library found in " ${BOOST_ROOT})
                include_directories(${Boost_INCLUDE_DIRS})
                link_directories( ${Boost_LIBRARY_DIRS} )
        else(Boost_FOUND)
                message(STATUS "Boost library not found. Boost-dependent headers will NOT be available")
        endif(Boost_FOUND)
endif(Boost_FOUND)

#### Coin3D package ###############
# graphics library
find_package( Coin REQUIRED )
if( COIN_LIBRARY_FOUND )
   message( STATUS "Using Coin library")
   add_definitions( -DCOIN_DLL )
   include_directories( ${COIN_INCLUDE_DIR} )
else( COIN_LIBRARY_FOUND )
   message(SEND_ERROR "Kautham needs Coin library")
endif(COIN_LIBRARY_FOUND )

#### Assimp ######
find_package( Assimp )
if( ASSIMP_FOUND )
   message( STATUS "assimp Package FOUND")
set(EXTRA_LIBS ${EXTRA_LIBS} ${ASSIMP_LIBRARIES} )
   add_definitions( -DKAUTHAM_USE_ASSIMP )
else( ASSIMP_FOUND )
   message( STATUS "assimp Package NOT FOUND")
endif( ASSIMP_FOUND )

find_package( Armadillo )
if( ARMADILLO_FOUND )
   message( STATUS "armadillo Package FOUND")
   add_definitions( -DKAUTHAM_USE_ARMADILLO )
else( ARMADILLO_FOUND )
   message( STATUS "armadillo Package NOT FOUND. Planners using PCA will be disabled.")
endif( ARMADILLO_FOUND )

find_package(ompl)
if( OMPL_FOUND )
   message( STATUS "OMPL Package FOUND: setting -DKAUTHAM_USE_OMPL flag")
   message( STATUS "OMPL lib is: " ${OMPL_LIBRARIES})
   message( STATUS "OMPL include is: " ${OMPL_INCLUDE_DIRS})
   add_definitions( -DKAUTHAM_USE_OMPL )
endif()

if( KAUTHAM_IOC )
   message( STATUS "Use IOC planners: setting -DKAUTHAM_USE_IOC flag")
   add_definitions( -DKAUTHAM_USE_IOC )
endif( KAUTHAM_IOC )

if(KAUTHAM_OPENDE)
   find_package(OpenDE REQUIRED)
   if(ODE_FOUND)
      message( STATUS "ODE Package FOUND: setting -DKAUTHAM_USE_ODE flag")
      add_definitions( -DKAUTHAM_USE_ODE )
   endif(ODE_FOUND)
endif(KAUTHAM_OPENDE)



add_subdirectory( applications)
add_subdirectory( planner )
add_subdirectory( problem )
add_subdirectory( sampling )
#                      ${GDIAM_DIR}
#                      ${Armadillo_INCLUDE_DIR})

#set(KAUTHAM_SRC "")
#set(KAUTHAM_HRC "")

#file( GLOB SRC_FILES problem/*.cpp )
#file( GLOB HDR_FILES problem/*.h* external/*.h* )


add_definitions( -DKAUTHAM_COLLISION_PQP ) # Remove it if you don't want to create a PQP collision models.
message( STATUS "Kautham sources at ${CMAKE_CURRENT_SOURCE_DIR} to create the library: ${KAUTHAM_SRC}")

add_library(kautham SHARED ${KAUTHAM_SRC} )

target_link_libraries(kautham ${EXTRA_LIBS} ${Boost_LIBRARIES} )
# 			sampling
# 			libPQP
# 			libgdiam
# 			${PUGI_XML_LIBRARIES}
# 			${OMPL_LIBRARIES}
# 			${ARMADILLO_LIBRARIES}
# 			${ODE_LIBRARY}
# 			libDNN
# 			libkin
# 			libkthutil
#
# 			${EXTRA_LIBS})

