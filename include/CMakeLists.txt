# configure a header file to pass some of the CMake settings to the source code
configure_file ("kauthamConfig.h.in" "${CMAKE_BINARY_DIR}/include/kautham/kauthamConfig.h")


# Copy Help file
install(DIRECTORY kautham DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES ${CMAKE_BINARY_DIR}/include/kautham/kauthamConfig.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/kautham/)
