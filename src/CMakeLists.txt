#### Adding subdirectories

include_directories( ${CMAKE_SOURCE_DIR}/include)
set(mt_DIR ${CMAKE_SOURCE_DIR}/src/util/libmt)
message (STATUS "Adding libmt directory ${mt_DIR}")
include_directories( ${mt_DIR} )
include_directories( ${CMAKE_SOURCE_DIR}/src )  # for external

set ( kin_DIR ${CMAKE_CURRENT_SOURCE_DIR}/util/libkin )
set ( kthutil_DIR ${CMAKE_CURRENT_SOURCE_DIR}/util/kthutil )
set ( mt_DIR ${CMAKE_CURRENT_SOURCE_DIR}/util/libmt )

##### pugixml ########
find_package( PugiXML )
if(PUGI_XML_FOUND)
   message( STATUS "PugiXML FOUND, using it")
   include_directories( ${PUGI_XML_INCLUDE_DIR})
else(PUGI_XML_FOUND)
   message(STATUS "PugiXML not found, using internal copy")
   add_subdirectory ( external/pugixml )
   include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/external/pugixml )
   set(PUGI_XML_LIBRARIES libpugixml)
endif(PUGI_XML_FOUND)
set(EXTRA_LIBS ${EXTRA_LIBS} ${PUGI_XML_LIBRARIES} PARENT_SCOPE  )

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
                set(EXTRA_LIBS ${EXTRA_LIBS} ${Boost_LIBRARIES})
                link_directories( ${Boost_LIBRARY_DIRS} )
        else(Boost_FOUND)
                message(STATUS "Boost library not found. Boost-dependent headers will NOT be available")
        endif(Boost_FOUND)
endif(Boost_FOUND)

add_subdirectory( util )

include_directories( ${CMAKE_SOURCE_DIR}/include)
#                       ${kthutil_DIR}
#                       ${mt_DIR}
#                       ${kin_DIR})                   

add_subdirectory ( external )

#### Coin3D package ###############
# graphics library
find_package( Coin REQUIRED )
if( COIN_LIBRARY_FOUND )
   message( STATUS "Using Coin library")
   add_definitions( -DCOIN_DLL )
   include_directories( ${COIN_INCLUDE_DIR} )
    set(EXTRA_LIBS ${EXTRA_LIBS} ${COIN_LIBRARY})
else( COIN_LIBRARY_FOUND )
   message(SEND_ERROR "Kautham needs Coin library")
endif(COIN_LIBRARY_FOUND )

#### Assimp ######
find_package( Assimp )
if( ASSIMP_FOUND )
   message( STATUS "assimp Package FOUND")
   set(EXTRA_LIBS ${EXTRA_LIBS} ${ASSIMP_LIBRARIES} )
   include_directories(${ASSIMP_INCLUDE_DIR})
   add_definitions( -DKAUTHAM_USE_ASSIMP )
else( ASSIMP_FOUND )
   message( STATUS "assimp Package NOT FOUND")
endif( ASSIMP_FOUND )

#### fcl ######
find_package( fcl )
if( FCL_FOUND )
   message( STATUS "fcl Package FOUND")
   set(EXTRA_LIBS ${EXTRA_LIBS} ${FCL_LIBRARIES} )
   include_directories(${FCL_INCLUDE_DIR})
   add_definitions( -DKAUTHAM_USE_FCL )
else( FCL_FOUND )
   message( STATUS "fcl Package NOT FOUND. PQP will be used for collision checking")
   add_definitions( -DKAUTHAM_USE_PQP )
endif( FCL_FOUND )

#### Armadillo ######
find_package( Armadillo REQUIRED)
if( ARMADILLO_FOUND )
   message( STATUS "armadillo Package FOUND")
   include_directories(${ARMADILLO_INCLUDE_DIRS})
   set(EXTRA_LIBS ${EXTRA_LIBS} ${ARMADILLO_LIBRARIES})
   add_definitions( -DKAUTHAM_USE_ARMADILLO )
else( ARMADILLO_FOUND )
   message( SEND_ERROR  "armadillo Package NOT FOUND. Planners using PCA will be disabled. Kautham needs it by now")
endif( ARMADILLO_FOUND )

find_package(ompl)
if( OMPL_FOUND )
   message( STATUS "OMPL Package FOUND: setting -DKAUTHAM_USE_OMPL flag")
   message( STATUS "OMPL lib is: " ${OMPL_LIBRARIES})
   message( STATUS "OMPL include is: " ${OMPL_INCLUDE_DIRS})
   include_directories(${OMPL_INCLUDE_DIRS})
   set(EXTRA_LIBS ${EXTRA_LIBS} ${OMPL_LIBRARIES})
   add_definitions( -DKAUTHAM_USE_OMPL )
endif()

find_package(Pthreads)
if(PTHREADS_FOUND)
   message( STATUS "Pthreads Package FOUND:")
   include_directories(${PTHREADS_INCLUDE_DIR})
   set(EXTRA_LIBS ${EXTRA_LIBS} ${PTHREADS_LIBRARY})
   add_definitions( ${PTHREADS_DEFINITIONS} )
endif()

if(KAUTHAM_OPENDE)
   find_package(OpenDE REQUIRED)
   if(ODE_FOUND)
      message( STATUS "ODE Package FOUND: setting -DKAUTHAM_USE_ODE flag")
      include_directories(${ODE_INCLUDE_DIR})
      set(EXTRA_LIBS ${EXTRA_LIBS} ${ODE_LIBRARY})
      add_definitions( -DKAUTHAM_USE_ODE )
   endif(ODE_FOUND)
endif(KAUTHAM_OPENDE)

if( KAUTHAM_IOC )
   message( STATUS "Use IOC planners: setting -DKAUTHAM_USE_IOC flag")
   add_definitions( -DKAUTHAM_USE_IOC )
endif( KAUTHAM_IOC )


# Adding subdirectories

add_subdirectory( applications)
add_subdirectory( planner )
add_subdirectory( problem )
add_subdirectory( sampling )

#message( STATUS "Kautham sources at ${CMAKE_CURRENT_SOURCE_DIR} to create the library: ${KAUTHAM_SRC}")

add_library(libkautham SHARED ${KAUTHAM_SRC} ${CMAKE_CURRENT_SOURCE_DIR}/applications/console/kauthamshell.cpp)
set_target_properties( libkautham PROPERTIES
  SOVERSION "${KAUTHAM_VERSION_MAJOR}.${KAUTHAM_VERSION_MINOR}"
  VERSION "${KAUTHAM_VERSION}"
  OUTPUT_NAME kautham)
  

target_link_libraries(libkautham ${EXTRA_LIBS})

# extra libraries should contain sampling libPQP libgdiam PUGI OMPL ARMADILLO ODE libDNN libkin libkthutil 

set(KAUTHAM_DIRNAME_lib ${CMAKE_INSTALL_LIBDIR})
set(KAUTHAM_DIRNAME_include "${CMAKE_INSTALL_PREFIX}/include/kautham")


# Headers install
install(FILES ${KAUTHAM_HDRS} DESTINATION include/kautham)

install(TARGETS ${KAUTHAM_TARGETS} EXPORT KAUTHAMExports
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}   
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}   
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include/fril)

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/kauthamConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/kauthamConfig.cmake"   
  INSTALL_DESTINATION  ${KAUTHAM_DIRNAME_lib}/cmake/kautham
  PATH_VARS 
   KAUTHAM_DIRNAME_lib
   KAUTHAM_DIRNAME_include)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/kauthamConfigVersion.cmake"
  VERSION ${KAUTHAM_VERSION}
  COMPATIBILITY AnyNewerVersion)

#install( EXPORT KAUTHAMExports
#         DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/kautham)

install( FILES ${CMAKE_CURRENT_BINARY_DIR}/kauthamConfig.cmake
       ${CMAKE_CURRENT_BINARY_DIR}/kauthamConfigVersion.cmake 
       DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/kautham)
   
install(TARGETS libkautham DESTINATION ${CMAKE_INSTALL_LIBDIR})
