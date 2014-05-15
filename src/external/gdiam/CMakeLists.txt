file(GLOB SRC_FILES *.c* )
file(GLOB HDR_FILES *.h*)

set(KAUTHAM_SRC ${KAUTHAM_SRC} ${SRC_FILES} PARENT_SCOPE )

# add_library(libgdiam STATIC ${SRC_FILES} ${HDR_FILES} )
# if(UNIX)
#   set_target_properties( libgdiam PROPERTIES
#         LINKER_LANGUAGE CXX
#         COMPILE_FLAGS -fPIC
#         LINK_FLAGS -Wl
#         OUTPUT_NAME gdiam )
# endif(UNIX)
