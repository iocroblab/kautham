file( GLOB SRC_FILES *.cpp )
file( GLOB HDR_FILES *.h* )

set(KAUTHAM_SRC ${KAUTHAM_SRC} ${SRC_FILES} PARENT_SCOPE)

# add_library(libpugixml STATIC ${SRC_FILES} ${HDR_FILES} )
# if(UNIX)
#   set_target_properties( libpugixml PROPERTIES
#         LINKER_LANGUAGE CXX
#         COMPILE_FLAGS -fPIC
#         LINK_FLAGS -Wl
#         OUTPUT_NAME pugixml )
# endif(UNIX)
#
