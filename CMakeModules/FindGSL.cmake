# - Find GSL
# Find the GNU Scientific Library (GSL) includes and library
#
# This module defines
#  GSL_FOUND
#  GSL_LIBRARIES
#  GSL_CBLAS_LIBRARIES
#  GSL_INCLUDE_DIR
#  GSL_VERSION
#

if (GSL_INCLUDE_DIR AND GSL_LIBRARIES AND GSL_CBLAS_LIBRARIES AND GSL_VERSION)

	# Already in cache
	set(GSL_FOUND TRUE)

else (GSL_INCLUDE_DIR AND GSL_LIBRARIES AND GSL_CBLAS_LIBRARIES AND GSL_VERSION)

	find_library (GSL_LIBRARIES gsl)

	find_library (GSL_CBLAS_LIBRARIES gslcblas)

	find_path (GSL_INCLUDE_DIR gsl_multimin.h
		/usr/include/gsl
		/usr/local/include/gsl
        ${CMAKE_INCLUDE_PATH}/gsl
	)

	find_program (GSL_CONFIG gsl-config)

	if (GSL_CONFIG)
		exec_program (${GSL_CONFIG} ARGS "--version" OUTPUT_VARIABLE gsl_version)
	#	exec_program (${GSL_CONFIG} ARGS "--cflags" OUTPUT_VARIABLE gsl_include_dir)
	#	exec_program (${GSL_CONFIG} ARGS "--libs" OUTPUT_VARIABLE gsl_libraries)
	
	#	string (REGEX REPLACE "-I([^ ]*)" "\\1" GSL_INCLUDE_DIR "${gsl_include_dir}")
	#	string (REGEX REPLACE "-L([^ ]*)" "\\1" GSL_LIBRARIES "${gsl_libraries}")
		set (GSL_VERSION ${gsl_version} CACHE STRING "GNU Scientific Library Version")
		# TODO check version! 1.6 suffices?
	endif (GSL_CONFIG)


	#
	# everything necessary found?
	#
	if (GSL_LIBRARIES AND GSL_CBLAS_LIBRARIES AND GSL_INCLUDE_DIR)
		set (GSL_FOUND TRUE)
	else (GSL_LIBRARIES AND GSL_CBLAS_LIBRARIES AND GSL_INCLUDE_DIR)
		set (GSL_FOUND FALSE)
	endif (GSL_LIBRARIES AND GSL_CBLAS_LIBRARIES AND GSL_INCLUDE_DIR)

endif (GSL_INCLUDE_DIR AND GSL_LIBRARIES AND GSL_CBLAS_LIBRARIES AND GSL_VERSION)


#
# output status
#
if (GSL_FOUND)
     if (NOT GSL_FIND_QUIETLY)
	message (STATUS "Found GNU Scientific Library ${GSL_VERSION}: ${GSL_INCLUDE_DIR} ${GSL_LIBRARIES};${GSL_CBLAS_LIBRARIES}")
     endif(NOT GSL_FIND_QUIETLY)
else (GSL_FOUND)
     if (GSL_FIND_REQUIRED)
	message (STATUS "GNU Scientific Library not found. "
			"KSpread's solver plugin won't be compiled.")
     endif (GSL_FIND_REQUIRED)
endif (GSL_FOUND)

