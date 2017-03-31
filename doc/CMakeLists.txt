##############################################################################
# Documentation
# Tip from
# http://mementocodex.wordpress.com/2013/01/19/how-to-generate-code-documentation-with-doxygen-and-cmake-a-slightly-improved-approach/

if(KAUTHAM_DOCUMENTATION)
  find_package(Doxygen)  
  if(NOT DOXYGEN_FOUND)  
      message(STATUS "Doxygen is needed to build the documentation. Disabling create documentation")
      set(KAUTHAM_DOCUMENTATION OFF)
  else(DOXYGEN_FOUND)
      set( doxyfile_in          ${PROJECT_SOURCE_DIR}/doc/Doxyfile.in)
      set( doxyfile             ${PROJECT_BINARY_DIR}/doc/Doxyfile              )
      set( doxy_html_index_file ${PROJECT_BINARY_DIR}/doc/kautham-api/html/index.html )
      set( doxy_output_root     ${PROJECT_BINARY_DIR}/doc/kautham-api                 ) # Pasted into Doxyfile.in
      set( doxy_input           ${PROJECT_SOURCE_DIR}/src                   ) # Pasted into Doxyfile.in
      #  set( doxy_extra_files     ${CMAKE_CURRENT_SOURCE_DIR}/mainpage.dox    ) # 
      # Pasted into Doxyfile.in

      configure_file( ${doxyfile_in} ${doxyfile} @ONLY )

      add_custom_command( OUTPUT ${doxy_html_index_file}
                           COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
                           # The following should be ${doxyfile} only but it
                           # will break the dependency.
                           # The optimal solution would be creating a
                           # custom_command for ${doxyfile} generation
                           # but I still have to figure out how...
                           MAIN_DEPENDENCY ${doxyfile} ${doxyfile_in}
                           DEPENDS libkautham 
                           COMMENT "Generating HTML documentation")

      add_custom_target( doc DEPENDS ${doxy_html_index_file} )

      install( DIRECTORY ${PROJECT_BINARY_DIR}/doc/kautham-api/html 
               DESTINATION share/kautham/doc OPTIONAL)
  endif()
endif()
##############################################################################
