

FILE( GLOB QT_MOC_HDR *.h* )
QT4_WRAP_CPP( QT_MOC_SRC ${QT_MOC_HDR} )

FILE( GLOB FORMS_UI *.ui )
QT4_WRAP_UI( QTUI_H_SRC ${FORMS_UI} )

FILE( GLOB SRC_FILES *.cpp )
FILE( GLOB HDR_FILES *.h )
FILE( GLOB RES_FILES *.qrc )

ADD_DEFINITIONS( -DSOQT_DLL -DCOIN_DLL )

INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}
                     ${CMAKE_SOURCE_DIR}/libmt
                     ${SOQT_INCLUDE_DIR}
                     ${QT_INCLUDE_DIR}
                     ${CMAKE_CURRENT_BINARY_DIR}
                     ${CMAKE_CURRENT_BINARY_DIR}/libsexternal/libguibro )

QT4_ADD_RESOURCES( CPP_RES_FILES ${RES_FILES} )

ADD_LIBRARY( libguibro ${CPP_RES_FILES} ${SRC_FILES} ${HDR_FILES} ${QT_MOC_HDR}
                                        ${QT_MOC_SRC} ${QTUI_H_SRC} )
TARGET_LINK_LIBRARIES( libguibro ${QT_LIBRARIES} libproblem libplanner
                       libsampling libkthutil ${SOQT_LIBRARY} ${COIN_LIBRARY} )




#FILE(GLOB SRC_FILES *.c* )
#FILE(GLOB HDR_FILES *.h*)
#
#
#ADD_LIBRARY(libguibro STATIC ${SRC_FILES} ${HDR_FILES} )
