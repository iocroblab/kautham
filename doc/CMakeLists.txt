
##############################################################################
# Documentation
# Tip from
# http://mementocodex.wordpress.com/2013/01/19/how-to-generate-code-documentation-with-doxygen-and-cmake-a-slightly-improved-approach/

if(KAUTHAM_DOCUMENTATION_API)
  find_package(Doxygen)  
  if(NOT DOXYGEN_FOUND)  
      message(STATUS "Doxygen is needed to build the documentation. Disabling create documentation")
      set(KAUTHAM_DOCUMENTATION_API OFF)
  else(DOXYGEN_FOUND)
      add_subdirectory(doxygen)
  endif(NOT DOXYGEN_FOUND)
endif(KAUTHAM_DOCUMENTATION_API)
  
