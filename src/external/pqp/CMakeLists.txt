# libPQP


file( GLOB SRC_FILES *.cpp )
file( GLOB HDR_FILES *.h* )

set(KAUTHAM_SRC ${KAUTHAM_SRC} ${SRC_FILES} PARENT_SCOPE )

# add_library( libPQP  STATIC ${SRC_FILES} ${HDR_FILES} )
#
# if(UNIX)
#   set_target_properties( libPQP PROPERTIES
#         LINKER_LANGUAGE CXX
#         COMPILE_FLAGS -fPIC
#         LINK_FLAGS -Wl
#         OUTPUT_NAME pqp )
# endif(UNIX)

# add_library(libPQP
		# Build.h
		# BV.h
		# MatVec.h
		# PQP.cpp
		# PQP_Compile.h
		# RectDist.h
		# TriDist.cpp
		# Build.cpp
		# BV.cpp
		# BVTQ.h
		# GetTime.h
		# OBB_Disjoint.h
		# PQP.h
		# PQP_Internal.h
		# Tri.h
		# TriDist.h)


