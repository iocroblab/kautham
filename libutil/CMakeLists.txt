# Utilities library. This library contains the general definitions of the Kautham and the KauthamObject.

INCLUDE_DIRECTORIES( pugixml ${Boost_INCLUDE_DIRS} )
FILE(GLOB SRC_FILES *.c* )
FILE(GLOB HDR_FILES *.h*)
FILE(GLOB PUGI_SRC_FILES pugixml/*.c* )
FILE(GLOB PUGI_HDR_FILES pugixml/*.h*)

FIND_PACKAGE( Qt4 REQUIRED )
  IF( QT4_FOUND )
    SET( QT_USE_QTMAIN TRUE )
    SET( QT_USE_QTNETWORK TRUE )
    INCLUDE ( ${QT_USE_FILE} )
    ADD_DEFINITIONS( ${QT_DEFINITIONS} )
  ENDIF( QT4_FOUND )

ADD_LIBRARY(libkthutil STATIC ${SRC_FILES} ${HDR_FILES} ${PUGI_SRC_FILES} ${PUGI_HDR_FILES} )
TARGET_LINK_LIBRARIES( libkthutil ${QT_LIBRARIES} )

IF(UNIX)
  SET_TARGET_PROPERTIES( libkthutil PROPERTIES OUTPUT_NAME kthutil )
ENDIF(UNIX)