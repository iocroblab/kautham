
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src)
INCLUDE_DIRECTORIES(${mt_DIR})


set(SRC_FILES benchmark.cpp
		kauthamplanner_console.cpp
		kauthamshell.cpp)


ADD_EXECUTABLE( KauthamConsole ${SRC_FILES} )
TARGET_LINK_LIBRARIES( KauthamConsole kautham ${COIN_LIBRARY})


