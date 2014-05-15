# LibDNN

#include_directories( .)
file( GLOB SRC_FILES *.c *.C )
file( GLOB HDR_FILES DNN/*.h )

set(KAUTHAM_SRC ${KAUTHAM_SRC} ${SRC_FILES} PARENT_SCOPE )

# add_library(libDNN STATIC ${SRC_FILES} ${HDR_FILES} )
#
# if(UNIX)
#   set_target_properties( libDNN PROPERTIES
# 	LINKER_LANGUAGE CXX
#   	COMPILE_FLAGS -fPIC
# 	LINK_FLAGS -Wl
# 	OUTPUT_NAME DNN )
# endif(UNIX)
