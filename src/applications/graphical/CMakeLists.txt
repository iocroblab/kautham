INCLUDE_DIRECTORIES( ${COIN_INCLUDE_DIR} )
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src)
INCLUDE_DIRECTORIES(${mt_DIR})

ADD_DEFINITIONS( ${QT_DEFINITIONS} )
##############################################################################
# START building Kautham GUI application
IF( KAUTHAM_GUI )
  MESSAGE( STATUS "making KAUTHAM GUI" )
ENDIF( KAUTHAM_GUI )

##############################################################################
 IF( SOQT_LIBRARY_FOUND )
    ADD_DEFINITIONS( -DSOQT_DLL -DCOIN_DLL )
    INCLUDE_DIRECTORIES( ${SOQT_INCLUDE_DIR} )
  ENDIF( SOQT_LIBRARY_FOUND )
##############################################################################

##############################################################################
# Creating executable
  FILE( GLOB RES_FILES *.rc )
  SET(kautham_RESOURCES kauthamRes.qrc)

  QT4_ADD_RESOURCES(kautham_RESOURCES_RCC ${kautham_RESOURCES})
  QT4_WRAP_CPP( QT_MOC_SRC kauthamgui.h 
		aboutwidget.h
		axis.h
        constrainedcontrolwidget.h
        controlwidget.h
        dofwidget.h
        gui.h
        invkinwidget.h
        kauthamgui.h
        kauthamwidget.h
        planners_toolbar.h
        plannerwidget.h
        sampleswidget.h
        streamlog.h
        ui_RobotSim.h
        viewertype.h

		OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)

  SET( SRC_FILES_GUI kauthamgui.cpp 
	constrainedcontrolwidget.cpp  
	dofwidget.cpp  
	invkinwidget.cpp
	kauthamwidget.cpp     
	plannerwidget.cpp
	controlwidget.cpp
	gui.cpp
	kauthamgui.cpp
	planners_toolbar.cpp
	sampleswidget.cpp)
  SET( HDR_FILES_GUI kauthamgui.h 
	aboutwidget.h
	axis.h
	constrainedcontrolwidget.h
	controlwidget.h
	dofwidget.h
	gui.h
	invkinwidget.h
	kauthamgui.h
	kauthamwidget.h
	planners_toolbar.h
	plannerwidget.h
	sampleswidget.h
	streamlog.h
	ui_RobotSim.h
	viewertype.h)

  ADD_EXECUTABLE(Kautham2  ${SRC_FILES_GUI} ${HDR_FILES_GUI} ${QT_MOC_SRC} ${kautham_RESOURCES_RCC} )
  TARGET_LINK_LIBRARIES( Kautham2 
		#${KC_EXTRA_LIBS}
					libDNN libPQP
					sampling
					problem libkthutil libkin
					libgdiam
					${SOQT_LIBRARY} ${COIN_LIBRARY}
					${QT_LIBRARIES}
					${PUGI_XML_LIBRARIES})
#  ADD_DEPENDENCIES(libDNN libPQP sampling problem kthutil libkin libgdiam)

# END building Kautham GUI application


INSTALL(TARGETS Kautham2 DESTINATION bin)



