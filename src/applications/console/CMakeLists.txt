
set(SRC_FILES benchmark.cpp
		kauthamplanner_console.cpp
		kauthamshell.cpp)

include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${mt_DIR}
                    ${OMPL_INCLUDE_DIRS})
if(KAUTHAM_OMPL)
   add_definitions( -DKAUTHAM_USE_OMPL )
   add_executable( kautham-console ${SRC_FILES} )
   target_link_libraries( kautham-console libkautham ${COIN_LIBRARY} ${Boost_LIBRARIES})
   add_dependencies(kautham-console libkautham)
   install(TARGETS kautham-console DESTINATION bin)
endif(KAUTHAM_OMPL)




