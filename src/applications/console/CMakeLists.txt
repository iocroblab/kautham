
set(SRC_FILES benchmark.cpp
		kauthamplanner_console.cpp
		kauthamshell.cpp)

include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${mt_DIR}
                    ${OMPL_INCLUDE_DIRS})
if(KAUTHAM_OMPL)
   add_definitions( -DKAUTHAM_USE_OMPL )
   add_executable( KauthamConsole ${SRC_FILES} )
   target_link_libraries( KauthamConsole kautham ${COIN_LIBRARY})
   add_dependencies(KauthamConsole kautham)
   install(TARGETS KauthamConsole DESTINATION bin)
endif(KAUTHAM_OMPL)




