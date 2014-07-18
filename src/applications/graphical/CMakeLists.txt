include_directories( ${COIN_INCLUDE_DIR} )
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${mt_DIR})

##############################################################################
# START Required packages for GUI
find_package( Qt4 REQUIRED QtCore QtGui QtXml )
if( QT4_FOUND )
   include(${QT_USE_FILE})
   add_definitions( ${QT_DEFINITIONS} )
endif( QT4_FOUND )

##############################################################################
find_package( SoQt4 REQUIRED )
if( SOQT_LIBRARY_FOUND )
   add_definitions( -DSOQT_DLL -DCOIN_DLL )
   include_directories( ${SOQT_INCLUDE_DIR} )
endif( SOQT_LIBRARY_FOUND )
# END Required packages for GUI
##############################################################################

if(KAUTHAM_OMPL)
   add_definitions( -DKAUTHAM_USE_OMPL )
endif(KAUTHAM_OMPL)
##############################################################################
# Creating executable
file( GLOB RES_FILES *.rc )
set(kautham_RESOURCES kauthamRes.qrc)

QT4_ADD_RESOURCES(kautham_RESOURCES_RCC ${kautham_RESOURCES})
QT4_WRAP_CPP( QT_MOC_SRC kauthamgui.h
         aboutwidget.h
         attachdialog.h
         constrainedcontrolwidget.h
         controlwidget.h
         deattachdialog.h
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
         attachdialog.cpp
         constrainedcontrolwidget.cpp
         deattachdialog.cpp
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
         attachdialog.h
         constrainedcontrolwidget.h
         controlwidget.h
         deattachdialog.h
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

add_executable(Kautham2  ${SRC_FILES_GUI} ${HDR_FILES_GUI} ${QT_MOC_SRC} ${kautham_RESOURCES_RCC} )
target_link_libraries( Kautham2 kautham ${SOQT_LIBRARY} ${COIN_LIBRARY} ${QT_LIBRARIES})

add_dependencies(Kautham2 kautham)

install(TARGETS Kautham2 DESTINATION bin)

# END building Kautham GUI application


