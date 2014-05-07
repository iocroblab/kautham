#######################
# Once you use this CMakeFile to configure the mt some variables will be set:
# MT_HEADERS
# ASSERT_HEADERS
# REL_HEADERS
# TTL_HEADERS
# UTIL_HEADERS
# You can use them in order to include this headers in your project.
#######################

##############################################################################
# Required packages
##############################################################################

find_package(Boost)
if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
  link_directories(${Boost_LIBRARY_DIRS})
  add_definitions(-DMT_USE_BOOST)
  message(STATUS "Boost library found. Boost-dependent headers will be available")
else(Boost_FOUND)
  remove_definitions(-DMT_USE_BOOST)
  message(STATUS "Boost library not found. Boost-dependent headers will NOT be available")
endif(Boost_FOUND)


##############################################################################
# Subdirectories
##############################################################################