# kautham utilities library. This library contains the general definitions of the Kautham and the KauthamObject.

FILE(GLOB SRC_FILES *.c* )
FILE(GLOB HDR_FILES *.h*)

ADD_LIBRARY(libkthutil STATIC ${SRC_FILES} ${HDR_FILES} )
