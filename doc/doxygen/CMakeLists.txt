##############################################################################
# Documentation
# Tip from
# http://mementocodex.wordpress.com/2013/01/19/how-to-generate-code-documentation-with-doxygen-and-cmake-a-slightly-improved-approach/

set( doxyfile_in          ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
set( doxyfile             ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile              )
set( doxy_html_index_file ${CMAKE_CURRENT_BINARY_DIR}/kautham-api/html/index.html )
set( doxy_output_root     ${CMAKE_CURRENT_BINARY_DIR}/kautham-api                 ) # Pasted into Doxyfile.in
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
         
##############################################################################
