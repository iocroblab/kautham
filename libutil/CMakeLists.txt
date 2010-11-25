# Utilities library. This library contains the general definitions of the Kautham and the KauthamObject.

INCLUDE_DIRECTORIES( pugixml ${Boost_INCLUDE_DIRS} )
FILE(GLOB SRC_FILES *.c* )
FILE(GLOB HDR_FILES *.h*)
FILE(GLOB PUGI_SRC_FILES pugixml/*.c* )
FILE(GLOB PUGI_HDR_FILES pugixml/*.h*)
ADD_LIBRARY(libkthutil STATIC ${SRC_FILES} ${HDR_FILES} ${PUGI_SRC_FILES} ${PUGI_HDR_FILES} )

IF(UNIX)
  SET_TARGET_PROPERTIES( libkthutil PROPERTIES OUTPUT_NAME kthutil )
ENDIF(UNIX)