# kautham utilities library. This library contains the general definitions of the Kautham and the KauthamObject.

file(GLOB SRC_FILES *.c* )
file(GLOB HDR_FILES *.h*)

set(KAUTHAM_SRC ${KAUTHAM_SRC} ${SRC_FILES} PARENT_SCOPE  )
# # add_library(libkthutil STATIC ${SRC_FILES} ${HDR_FILES} )
# # if(UNIX)
# #   set_target_properties( libkthutil PROPERTIES
# #         LINKER_LANGUAGE CXX
# #         COMPILE_FLAGS -fPIC
# #         LINK_FLAGS -Wl
# #         OUTPUT_NAME kthutil)
# # endif(UNIX)
