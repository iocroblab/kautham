##############################################################################
# START Required packages for GUI
find_package(Qt5 COMPONENTS Core Gui OpenGL Widgets Xml REQUIRED)
if(Qt5_FOUND)
   message(STATUS "Qt5 found")
   set(QT_LIBRARIES ${Qt5Core_LIBRARIES} ${Qt5Gui_LIBRARIES} ${Qt5OpenGL_LIBRARIES} ${Qt5Widgets_LIBRARIES} ${Qt5Xml_LIBRARIES})
   include_directories( ${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} ${Qt5OpenGL_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS} ${Qt5Xml_INCLUDE_DIRS})
endif( Qt5_FOUND )



##############################################################################
find_package( SoQt REQUIRED CONFIG)

if( SoQt_FOUND )
   add_definitions( ${SoQt_DEFINITIONS})
   include_directories( ${SoQt_INCLUDE_DIRS} )
endif( SoQt_FOUND )
#### Armadillo ######
find_package( Armadillo REQUIRED)
if( ARMADILLO_FOUND )
   message( STATUS "armadillo Package FOUND")
   add_definitions( -DKAUTHAM_USE_ARMADILLO )
else( ARMADILLO_FOUND )
   message( SEND_ERROR  "armadillo Package NOT FOUND. Bounding box button will not appear in the GUI.")
endif( ARMADILLO_FOUND )

# END Required packages for GUI
##############################################################################

if(KAUTHAM_OMPL)
   add_definitions( -DKAUTHAM_USE_OMPL )
endif(KAUTHAM_OMPL)
##############################################################################
# Creating executable
file( GLOB RES_FILES *.rc )
set(kautham_RESOURCES kauthamRes.qrc)

QT5_ADD_RESOURCES(kautham_RESOURCES_RCC ${kautham_RESOURCES})
QT5_WRAP_CPP( QT_MOC_SRC kauthamgui.h
         aboutwidget.h
         plannerparameters.h
         attachobjectdialog.h
         constrainedcontrolwidget.h
         controlwidget.h
         defaultpathdialog.h
         dofwidget.h
         gui.h
         invkinwidget.h
         kauthamgui.h
         kauthamwidget.h
         plannerwidget.h
         problemtreewidget.h
         sampleswidget.h
         OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)

set( SRC_FILES_GUI kauthamgui.cpp
         attachobjectdialog.cpp
         constrainedcontrolwidget.cpp
         defaultpathdialog.cpp
         dofwidget.cpp
         invkinwidget.cpp
         kauthamwidget.cpp
         plannerwidget.cpp
         controlwidget.cpp
         gui.cpp
         problemtreewidget.cpp
         sampleswidget.cpp)

set( HDR_FILES_GUI kauthamgui.h
         aboutwidget.h
         plannerparameters.h
         attachobjectdialog.h
         constrainedcontrolwidget.h
         controlwidget.h
         defaultpathdialog.h
         dofwidget.h
         gui.h
         invkinwidget.h
         kauthamgui.h
         kauthamwidget.h
         plannerwidget.h
         problemtreewidget.h
         sampleswidget.h
         ui_RobotSim.h)

add_executable(kautham-gui  ${SRC_FILES_GUI} ${HDR_FILES_GUI} ${QT_MOC_SRC} ${kautham_RESOURCES_RCC} )
target_link_libraries( kautham-gui libkautham ${SoQt_LIBRARIES} ${OPENGL_LIBRARIES} ${QT_LIBRARIES} ${Boost_LIBRARIES} ${PUGI_XML_LIBRARIES})

add_dependencies(kautham-gui libkautham)

install(TARGETS kautham-gui DESTINATION bin)

# END building Kautham GUI application
