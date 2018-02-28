##############################################################################
# START Required packages for GUI
find_package( Qt5Core COMPONENTS Core Widgets Xml)
# REQUIRED QtCore QtGui QtXml )
if( QT5_FOUND )
#   qt5_use_modules(
#   include(${QT_USE_FILE})
   add_definitions( ${QT_DEFINITIONS} )
endif( QT5_FOUND )

##############################################################################
find_package( SoQt REQUIRED )
if( SOQT_LIBRARY_FOUND )
   add_definitions( -DSOQT_DLL -DCOIN_DLL )
   include_directories( ${SOQT_INCLUDE_DIR} )
endif( SOQT_LIBRARY_FOUND )

#if (BZIP2_FOUND)
#  include_directories(${BZIP_INCLUDE_DIRS})
#  target_link_libraries (helloworld ${BZIP2_LIBRARIES})

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
target_link_libraries( kautham-gui libkautham ${SOQT_LIBRARY} Qt5::Widgets ${Boost_LIBRARIES} ${PUGI_XML_LIBRARIES})

add_dependencies(kautham-gui libkautham)

install(TARGETS kautham-gui DESTINATION bin)

# END building Kautham GUI application
