# this file makes the libiocplanner library.
set(SRC_FILES gridplanner.cpp
              HFplanner.cpp
              mygridplanner.cpp
              NF1planner.cpp)

include_directories( ${mt_DIR} )
			
file( GLOB SRC_IOC_FILES *.cpp )
file( GLOB HDR_IOC_FILES *.h )

set( KAUTHAM_SRC ${KAUTHAM_SRC} ${SRC_IOC_FILES} PARENT_SCOPE )



