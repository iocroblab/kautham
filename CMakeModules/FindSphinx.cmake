# - This module looks for Sphinx
# Find the Sphinx documentation generator
#
# This modules defines
#  SPHINX_EXECUTABLE
#  SPHINX_FOUND

# From here
# https://cmake.org/Bug/file_download.php?file_id=4176&type=bug

find_program(SPHINX_EXECUTABLE NAMES sphinx-build
  DOC "Sphinx documentation generator"
)

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(Sphinx DEFAULT_MSG
  SPHINX_EXECUTABLE
)


option( SPHINX_HTML_OUTPUT "Build a single HTML with the whole content." ON )
option( SPHINX_DIRHTML_OUTPUT "Build HTML pages, but with a single directory per document." OFF )
option( SPHINX_HTMLHELP_OUTPUT "Build HTML pages with additional information for building a documentation collection in htmlhelp." OFF )
option( SPHINX_devhelp_OUTPUT "Build HTML pages with additional information for building a documentation collection in qthelp." OFF )
option( SPHINX_DEVHELP_OUTPUT "Build HTML pages with additional information for building a documentation collection in devhelp." OFF )
option( SPHINX_EPUB_OUTPUT "Build HTML pages with additional information for building a documentation collection in epub." OFF )
option( SPHINX_LATEX_OUTPUT "Build LaTeX sources that can be compiled to a PDF document using pdflatex." OFF )
option( SPHINX_MAN_OUTPUT "Build manual pages in groff format for UNIX systems." OFF )
option( SPHINX_TEXT_OUTPUT "Build plain text files." OFF )


mark_as_advanced(
  SPHINX_EXECUTABLE
  SPHINX_HTML_OUTPUT
  SPHINX_DIRHTML_OUTPUT
  SPHINX_HTMLHELP_OUTPUT
  SPHINX_devhelp_OUTPUT
  SPHINX_DEVHELP_OUTPUT
  SPHINX_EPUB_OUTPUT
  SPHINX_LATEX_OUTPUT
  SPHINX_MAN_OUTPUT
  SPHINX_TEXT_OUTPUT
)

function( Sphinx_add_target target_name builder conf source destination )
  add_custom_target( ${target_name} ALL
    COMMAND ${SPHINX_EXECUTABLE} -b ${builder}
    -c ${conf}
    ${source}
    ${destination}
    COMMENT "Generating sphinx documentation: ${builder}"
    )

  set_property(
    DIRECTORY APPEND PROPERTY
    ADDITIONAL_MAKE_CLEAN_FILES
    ${destination}
    )
endfunction()

function( Sphinx_add_targets target_base_name conf source base_destination )

  set( _dependencies )

  foreach( arg IN LISTS ARGN )
    set( _dependencies ${_dependencies} ${arg} )
  endforeach()

  if( ${SPHINX_HTML_OUTPUT} )
    Sphinx_add_target( ${target_base_name}_html html ${conf} ${source} ${base_destination}/html )

    add_dependencies( ${target_base_name}_html _dependencies )
  endif()

  if( ${SPHINX_DIRHTML_OUTPUT} )
    Sphinx_add_target( ${target_base_name}_dirhtml dirhtml ${conf} ${source} ${base_destination}/dirhtml )

    add_dependencies( ${target_base_name}_dirhtml _dependencies )
  endif()

  if( ${SPHINX_QTHELP_OUTPUT} )
    Sphinx_add_target( ${target_base_name}_qthelp qthelp ${conf} ${source} ${base_destination}/qthelp )

    add_dependencies( ${target_base_name}_qthelp _dependencies )
  endif()

  if( ${SPHINX_DEVHELP_OUTPUT} )
    Sphinx_add_target( ${target_base_name}_devhelp devhelp ${conf} ${source} ${base_destination}/devhelp )

    add_dependencies( ${target_base_name}_devhelp _dependencies )
  endif()

  if( ${SPHINX_EPUB_OUTPUT} )
    Sphinx_add_target( ${target_base_name}_epub epub ${conf} ${source} ${base_destination}/epub )

    add_dependencies( ${target_base_name}_epub _dependencies )
  endif()

  if( ${SPHINX_LATEX_OUTPUT} )
    Sphinx_add_target( ${target_base_name}_latex latex ${conf} ${source} ${base_destination}/latex )

    add_dependencies( ${target_base_name}_latex _dependencies )
  endif()

  if( ${SPHINX_MAN_OUTPUT} )
    Sphinx_add_target( ${target_base_name}_man man ${conf} ${source} ${base_destination}/man )

    add_dependencies( ${target_base_name}_man _dependencies )
  endif()

  if( ${SPHINX_TEXT_OUTPUT} )
    Sphinx_add_target( ${target_base_name}_text text ${conf} ${source} ${base_destination}/text )

    add_dependencies( ${target_base_name}_text _dependencies )
  endif()

  if( ${BUILD_TESTING} )
    sphinx_add_target( ${target_base_name}_linkcheck linkcheck ${conf} ${source} ${base_destination}/linkcheck )

    add_dependencies( ${target_base_name}_linkcheck _dependencies )
  endif()
endfunction()
