#### Adding subdirectories

message(STATUS "We have this includes now ###################: ${ALL_INCLUDE_DIRS}")
list(APPEND ALL_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR})
##### pugixml ########
find_package( PugiXML REQUIRED)
if(PUGI_XML_FOUND)
   message( STATUS "PugiXML FOUND, using it")
   list(APPEND ALL_INCLUDE_DIRS ${PUGI_XML_INCLUDE_DIR})
   set(EXTRA_LIBS ${EXTRA_LIBS} ${PUGI_XML_LIBRARIES})
else(PUGI_XML_FOUND)
   message(FATAL_ERROR "PugiXML not found. Please install it")
endif(PUGI_XML_FOUND)

#### Coin3D package ###############
# graphics library
if(POLICY CMP0072)
  cmake_policy(SET CMP0072 NEW)
endif()

find_package(Coin REQUIRED)
if(Coin_FOUND)
  set(EXTRA_LIBS ${EXTRA_LIBS} ${Coin_LIBRARIES})
  list(APPEND ALL_INCLUDE_DIRS ${Coin_INCLUDE_DIR})
  add_definitions(${Coin_DEFINES})
else( Coin_FOUND )
   message(SEND_ERROR "Kautham needs Coin library")
endif(Coin_FOUND )

#### Assimp ######
find_package( Assimp )
if( ASSIMP_FOUND )
   message( STATUS "assimp Package FOUND")
   set(EXTRA_LIBS ${EXTRA_LIBS} ${ASSIMP_LIBRARIES} )
   list(APPEND ALL_INCLUDE_DIRS ${ASSIMP_INCLUDE_DIR})
   add_definitions( -DKAUTHAM_USE_ASSIMP )
else( ASSIMP_FOUND )
   message( STATUS "assimp Package NOT FOUND")
endif( ASSIMP_FOUND )

#### fcl ######
find_package( fcl REQUIRED CONFIG)
message( STATUS "fcl Package FOUND")
set(EXTRA_LIBS ${EXTRA_LIBS} ${FCL_LIBRARIES} )
list(APPEND ALL_INCLUDE_DIRS ${FCL_INCLUDE_DIRS})
message(STATUS "Internal libraries: ${FCL_LIBRARIES}")
add_definitions( -DKAUTHAM_USE_FCL )

#### Armadillo ######
find_package( Armadillo REQUIRED)
if( ARMADILLO_FOUND )
   message( STATUS "armadillo Package FOUND")
   list(APPEND ALL_INCLUDE_DIRS ${ARMADILLO_INCLUDE_DIRS})
   set(EXTRA_LIBS ${EXTRA_LIBS} ${ARMADILLO_LIBRARIES})
   add_definitions( -DKAUTHAM_USE_ARMADILLO )
else( ARMADILLO_FOUND )
   message( SEND_ERROR  "armadillo Package NOT FOUND. Planners using PCA will be disabled. Kautham needs it by now")
endif( ARMADILLO_FOUND )

find_package(Pthreads)
if(PTHREADS_FOUND)
   message( STATUS "Pthreads Package FOUND:")
   list(APPEND ALL_INCLUDE_DIRS ${PTHREADS_INCLUDE_DIR})
   set(EXTRA_LIBS ${EXTRA_LIBS} ${PTHREADS_LIBRARY})
   add_definitions( ${PTHREADS_DEFINITIONS} )
endif()

if(KAUTHAM_OPENDE)
   find_package(OpenDE REQUIRED)
   find_package(Drawstuff)
   ##### drawstuff ########
   #  DRAWSTUFF_LIBRARY     = location of drawstuff library
   #  DRAWSTUFF_INCLUDE_DIR = include path for drawstuff
   if(DRAWSTUFF_INCLUDE_DIR)
        message( STATUS "Drawstuff found, using it")
   else(DRAWSTUFF_INCLUDE_DIR)
         message(STATUS "DrawStuff not found, using internal copy")
         find_package(X11)
         if(X11_FOUND)
            message(STATUS "DrawStuff will be built and linked against X11")
            add_subdirectory ( ${CMAKE_CURRENT_SOURCE_DIR}/external/drawstuff)
            list(APPEND ALL_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/external/drawstuff/include)
            set(DRAWSTUFF_LIBRARY drawstuff)
            set(EXTRA_LIBS ${EXTRA_LIBS} ${X11_LIBRARIES})
         else(X11_FOUND)
            message(STATUS "DrawStuff cannot be built or used")
         endif(X11_FOUND)
   endif(DRAWSTUFF_INCLUDE_DIR)

   if(ODE_FOUND)
      message( STATUS "ODE Package FOUND: setting -DKAUTHAM_USE_ODE flag")
      list(APPEND ALL_INCLUDE_DIRS ${ODE_INCLUDE_DIR})
      set(EXTRA_LIBS ${EXTRA_LIBS} ${ODE_LIBRARY})
      add_definitions( -DKAUTHAM_USE_ODE )
   endif(ODE_FOUND)
endif(KAUTHAM_OPENDE)

if( KAUTHAM_IOC )
   message( STATUS "Use IOC planners: setting -DKAUTHAM_USE_IOC flag")
   add_definitions( -DKAUTHAM_USE_IOC )
endif( KAUTHAM_IOC )


# Adding subdirectories

add_subdirectory( planner )
add_subdirectory( problem )
add_subdirectory( sampling )
add_subdirectory( util )
add_subdirectory( external )

#message( STATUS "Kautham sources at ${CMAKE_CURRENT_SOURCE_DIR} to create the library: ${KAUTHAM_SRC}")

add_library(libkautham SHARED ${KAUTHAM_SRC} ${CMAKE_CURRENT_SOURCE_DIR}/kauthamshell.cpp)
add_library(Kautham::kautham ALIAS libkautham)

# Set include directories for the library
target_include_directories(libkautham
   PUBLIC
        $<BUILD_INTERFACE:${INCLUDE_DIR}>
        $<INSTALL_INTERFACE:include>
        ${OMPL_INCLUDE_DIRS}
   PRIVATE
        ${ALL_INCLUDE_DIRS}
)
# compile features 
target_compile_features(libkautham PUBLIC cxx_std_17)

# Add linker flags 
target_link_options(libkautham PUBLIC "-Wl,--no-undefined")
set_target_properties( libkautham PROPERTIES
  SOVERSION "${KAUTHAM_VERSION_MAJOR}.${KAUTHAM_VERSION_MINOR}"
  VERSION "${KAUTHAM_VERSION}"
  OUTPUT_NAME kautham)
  
message(STATUS "Internal libraries: ${EXTRA_LIBS}")

target_link_libraries(libkautham PRIVATE ${EXTRA_LIBS})

# extra libraries should contain sampling libPQP libgdiam PUGI OMPL ARMADILLO ODE libDNN libkin libkthutil 

set(KAUTHAM_DIRNAME_lib "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
set(KAUTHAM_DIRNAME_include "${CMAKE_INSTALL_PREFIX}/include")

message(STATUS "KAUTHAM_DIRNAME_lib: ${KAUTHAM_DIRNAME_lib}")
message(STATUS "KAUTHAM_DIRNAME_include: ${KAUTHAM_DIRNAME_include}")

# Headers install
install(FILES ${KAUTHAM_HDRS} DESTINATION include/kautham)

# Add version and configuration file generation
configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/kauthamConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/kauthamConfig.cmake"   
  INSTALL_DESTINATION  ${KAUTHAM_DIRNAME_lib}/cmake/kautham
  PATH_VARS 
   KAUTHAM_DIRNAME_lib
   KAUTHAM_DIRNAME_include
  NO_SET_AND_CHECK_MACRO
  NO_CHECK_REQUIRED_COMPONENTS_MACRO )

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/kauthamConfigVersion.cmake"
  VERSION ${KAUTHAM_VERSION_MAJOR}
  COMPATIBILITY AnyNewerVersion)

   
# Install the library
install(TARGETS libkautham
    EXPORT kauthamTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# Export the targets file
install(EXPORT kauthamTargets
    FILE kauthamTargets.cmake
    NAMESPACE Kautham::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/kautham
)

# Install the Config and Version files
install( FILES ${CMAKE_CURRENT_BINARY_DIR}/kauthamConfig.cmake
       ${CMAKE_CURRENT_BINARY_DIR}/kauthamConfigVersion.cmake 
       DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/kautham)
