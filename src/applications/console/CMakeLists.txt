#### Coin3D package ###############
# graphics library
FIND_PACKAGE( Coin REQUIRED )
#INCLUDE_DIRECTORIES( ${COIN_INCLUDE_DIR} )

# END General required packages
##############################################################################

##############################################################################
##############################################################################
# START Required packages when using OMPL planners
IF( KAUTHAM_OMPL )
    #### OMPL package ###############
    find_package(ompl)
    IF( OMPL_FOUND )
        MESSAGE( STATUS "OMPL Package FOUND: setting -DKAUTHAM_USE_OMPL flag")
        MESSAGE( STATUS "OMPL lib is: " ${OMPL_LIBRARIES})
        MESSAGE( STATUS "OMPL include is: "${OMPL_INCLUDE_DIRS})
        ADD_DEFINITIONS( -DKAUTHAM_USE_OMPL )
        MESSAGE( STATUS "OMPL VERSION is: " ${OMPL_VERSION})

        IF(OMPL_EXTENSION_OPENDE)

        IF(KAUTHAM_OPENDE)
        find_package(OpenDE REQUIRED)
            IF(ODE_FOUND)
                MESSAGE( STATUS "OMPL Package FOUND: setting -DKAUTHAM_USE_ODE flag")
                ADD_DEFINITIONS( -DKAUTHAM_USE_ODE )
            ENDIF(ODE_FOUND)
        ENDIF(KAUTHAM_OPENDE)

        ENDIF(OMPL_EXTENSION_OPENDE)
    ELSE( OMPL_FOUND )
        MESSAGE( STATUS "Error: OMPL Package NOT FOUND. OMPL planers will not be abaliable")
    ENDIF( OMPL_FOUND )
ENDIF( KAUTHAM_OMPL )
# END Required packages when using OMPL planners
##############################################################################


##############################################################################
##############################################################################
# START Required packages when using IOC planners
IF( KAUTHAM_IOC )
  #### GSL package ###############
  # maths lobrary used in libproblem for the SAH hand inverse kinematics
  FIND_PACKAGE( GSL )
  IF( GSL_FOUND )
    MESSAGE( STATUS "GSL FOUND.")
     ADD_DEFINITIONS( -DNDEBUG )
     ADD_DEFINITIONS( -DKAUTHAM_USE_GSL )
  ELSE( GSL_FOUND )
    MESSAGE( STATUS "GSL NOT FOUND. Hand inverse kinematics will be disabled")
  ENDIF( GSL_FOUND )

  #### Armadillo package ###############
  #used for PCA
  FIND_PACKAGE( Armadillo )
  IF( ARMADILLO_FOUND )
    MESSAGE( STATUS "armadillo Package FOUND")
    ADD_DEFINITIONS( -DKAUTHAM_USE_ARMADILLO )
  ELSE( ARMADILLO_FOUND )
    MESSAGE( STATUS "armadillo Package NOT FOUND. Planners using PCA will be disabled.")
  ENDIF( ARMADILLO_FOUND )
ENDIF( KAUTHAM_IOC )
# END Required packages when using IOC planners
##############################################################################


##############################################################################
# START Required packages for GUI
  FIND_PACKAGE( Qt4 REQUIRED QtCore QtGui QtXml )
  IF( QT4_FOUND )
    include(${QT_USE_FILE})
    #ADD_DEFINITIONS( ${QT_DEFINITIONS} )
  ENDIF( QT4_FOUND )



##############################################################################
# Building librarires
  INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR} )
  INCLUDE_DIRECTORIES( libsutil )
#  INCLUDE_DIRECTORIES( libsexternal )
  INCLUDE_DIRECTORIES( libsplanner )
  ADD_SUBDIRECTORY ( libsutil/libkin )
  ADD_SUBDIRECTORY ( libsutil/libann )
  ADD_SUBDIRECTORY ( libsutil/libkthutil )
  ADD_SUBDIRECTORY ( libsutil/libpqp )
  ADD_SUBDIRECTORY ( libsutil/libgdiam )
  ADD_SUBDIRECTORY ( libsplanner )
  ADD_SUBDIRECTORY ( libproblem )
  ADD_SUBDIRECTORY ( libsampling )


##############################################################################
# START building Kautham GUI application
IF( KAUTHAM_GUI )
  MESSAGE( STATUS "making KAUTHAM GUI" )


##############################################################################

  FIND_PACKAGE( SoQt4 REQUIRED )
  IF( SOQT_LIBRARY_FOUND )
    ADD_DEFINITIONS( -DSOQT_DLL -DCOIN_DLL )
    INCLUDE_DIRECTORIES( ${SOQT_INCLUDE_DIR} )
  ENDIF( SOQT_LIBRARY_FOUND )
# END Required packages for GUI
##############################################################################






##############################################################################
# Building librarires
#  IF( KAUTHAM_GUIBRO )
#    MESSAGE( STATUS "Creating libguibro")
#    INCLUDE_DIRECTORIES( libsexternal )
#    ADD_SUBDIRECTORY ( libsexternal/libguibro )
#    SET(KC_EXTRA_LIBS ${KC_EXTRA_LIBS}  libguibro)
#    MESSAGE( STATUS "KC_EXTRA_LIBSs: " ${KC_EXTRA_LIBS})
#  ENDIF( KAUTHAM_GUIBRO )

 ADD_SUBDIRECTORY ( libgui )



##############################################################################
# Creating executable
  FILE( GLOB RES_FILES *.rc )
  QT4_WRAP_CPP( QT_MOC_SRC libui/libguimain/application.h OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)

  SET( SRC_FILES_GUI libui/libguimain/kauthamgui.cpp libui/libguimain/kauthamgui.cpp )
  SET( HDR_FILES_GUI libui/libguimain/application.h )

  ADD_EXECUTABLE(Kautham2  ${SRC_FILES_GUI} ${HDR_FILES_GUI} ${QT_MOC_SRC} ${RES_FILES} )
  TARGET_LINK_LIBRARIES( Kautham2 ${KC_EXTRA_LIBS}
					libDNN libPQP
					libgui libsampling
					libproblem libkthutil libkin
					libgdiam
					${SOQT_LIBRARY} ${COIN_LIBRARY}
					${QT_LIBRARIES}
					${PUGI_XML_LIBRARIES})
  ADD_DEPENDENCIES(libDNN libPQP libgui libsampling libproblem libkthutil libkin libgdiam)
ENDIF( KAUTHAM_GUI )

# END building Kautham GUI application


##############################################################################
# START building Kautham Console application
IF( KAUTHAM_CONSOLE )

  MESSAGE( STATUS "making KAUTHAM CONSOLE" )

  #MPI Library used to turn the KauthamConsole parallel process.
  INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}  ${mt_DIR} ${COIN_INCLUDE_DIR} ${CMAKE_SOURCE_DIR}/libgui )
  #FIND_PACKAGE( MPI )
  IF( MPI_FOUND )
    INCLUDE_DIRECTORIES( ${MPI_INCLUDE_PATH} )
    ADD_DEFINITIONS( -DKAUTHAM_USE_MPI )
    MESSAGE( STATUS "MPI library found at" ${MPI_INCLUDE_PATH} )
    SET(KC_EXTRA_LIBS ${KC_EXTRA_LIBS} ${MPI_LIBRARIES} )
    MESSAGE( STATUS "MPI_LIBRARIES = " ${MPI_LIBRARIES} )
  ELSE( MPI_FOUND )
    MESSAGE( STATUS "MPI library not found" )
  ENDIF( MPI_FOUND )

  SET( HDR_FILES_GUI libui/libconsole/kauthamshell.h )
  SET( SRC_FILES libui/libconsole/kauthamshell.cpp libui/libconsole/kauthamplanner_console.cpp libsutil/libkthutil/kauthamobject.cpp )

    # Adding console bath mode executable
  ADD_EXECUTABLE( KauthamConsole ${SRC_FILES} ${HDR_FILES} ${RES_FILES} )
  TARGET_LINK_LIBRARIES( KauthamConsole libDNN libPQP
					libproblem libkthutil
					libkin libgdiam
					${KC_EXTRA_LIBS}
					${SOQT_LIBRARY} ${COIN_LIBRARY}
					${PUGI_XML_LIBRARIES} )

  MESSAGE( STATUS "KC_EXTRA_LIBS = " ${KC_EXTRA_LIBS} )
ENDIF( KAUTHAM_CONSOLE )

# END building Kautham Console application
##############################################################################


##############################################################################
# Documentation
# Tip from
# http://mementocodex.wordpress.com/2013/01/19/how-to-generate-code-documentation-with-doxygen-and-cmake-a-slightly-improved-approach/

option(BUILD_DOCUMENTATION "Create and install the HTML based API
documentation (requires Doxygen)" OFF)
IF(BUILD_DOCUMENTATION)
  FIND_PACKAGE(Doxygen)
  IF(NOT DOXYGEN_FOUND)
    MESSAGE(FATAL_ERROR
      "Doxygen is needed to build the documentation.")
  ENDIF()

  SET( doxyfile_in          ${CMAKE_CURRENT_SOURCE_DIR}/doc/Doxyfile.in     )
  SET( doxyfile             ${PROJECT_BINARY_DIR}/Doxyfile              )
  SET( doxy_html_index_file ${CMAKE_CURRENT_BINARY_DIR}/html/index.html )
  SET( doxy_output_root     ${CMAKE_CURRENT_BINARY_DIR}                 ) #  Pasted into Doxyfile.in
  SET( doxy_input           ${PROJECT_SOURCE_DIR}/src                   ) #  Pasted into Doxyfile.in
#  SET( doxy_extra_files     ${CMAKE_CURRENT_SOURCE_DIR}/mainpage.dox    ) #  Pasted into Doxyfile.in

  CONFIGURE_FILE( ${doxyfile_in} ${doxyfile} @ONLY )

  ADD_CUSTOM_COMMAND( OUTPUT ${doxy_html_index_file}
                      COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
                      # The following should be ${doxyfile} only but it
                      # will break the dependency.
                      # The optimal solution would be creating a
                      # custom_command for ${doxyfile} generation
                      # but I still have to figure out how...
                      MAIN_DEPENDENCY ${doxyfile} ${doxyfile_in}
                      DEPENDS Kautham2 ${doxy_extra_files}
                      COMMENT "Generating HTML documentation")

  ADD_CUSTOM_TARGET( doc DEPENDS ${doxy_html_index_file} )

  INSTALL( DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html DESTINATION share/doc)
ENDIF()
##############################################################################



##############################################################################
# Copy Help file

INSTALL(TARGETS Kautham2 DESTINATION bin)

INSTALL(DIRECTORY ${CMAKE_SOURCE_DIR}/demos DESTINATION share/kautham)



