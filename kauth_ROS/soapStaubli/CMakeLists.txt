
cmake_minimum_required(VERSION 2.6)
FILE( GLOB HDR_FILES *.h* )
FILE( GLOB SRC_FILES *.c* )	

ADD_LIBRARY(libsoapstaubli ${HDR_FILES} ${SRC_FILES} )

IF(UNIX)
  SET_TARGET_PROPERTIES( libsoapstaubli PROPERTIES OUTPUT_NAME soapstaubli )
ENDIF(UNIX)
