# kautham utilities library. This library contains the general definitions of the Kautham and the KauthamObject.

FILE(GLOB SRC_FILES *.c* )
FILE(GLOB HDR_FILES *.h*)

ADD_LIBRARY(libkthutil STATIC ${SRC_FILES} ${HDR_FILES} )
IF(UNIX)
  SET_TARGET_PROPERTIES( libkthutil PROPERTIES
        LINKER_LANGUAGE CXX
        COMPILE_FLAGS -fPIC
        LINK_FLAGS -Wl
        OUTPUT_NAME kthutil)
ENDIF(UNIX)
