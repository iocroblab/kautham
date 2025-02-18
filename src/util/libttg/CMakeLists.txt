# Library Time Trajectory Generator.
# This library allows to generate a trajectory from the given path and the maximum joint velocities and accelerations.

file(GLOB SRC_FILES *.cpp* )
file(GLOB HDR_FILES *.hpp*)

set(KAUTHAM_SRC ${KAUTHAM_SRC} ${SRC_FILES} PARENT_SCOPE  )

