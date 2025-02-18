set(SRC_FILES benchmark.cpp
              kauthamplanner_console.cpp)

include_directories(${mt_DIR}
                    ${OMPL_INCLUDE_DIRS})
if(KAUTHAM_OMPL)
   add_definitions( -DKAUTHAM_USE_OMPL )
   add_executable( kautham-console ${SRC_FILES} )

   target_include_directories(kautham-console PRIVATE ${OMPL_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/include)
   target_link_libraries( kautham-console libkautham ${Boost_LIBRARIES} ${PUGI_XML_LIBRARIES} ${OMPL_LIBRARIES})
   add_dependencies(kautham-console libkautham)
   install(TARGETS kautham-console DESTINATION bin)
endif(KAUTHAM_OMPL)




