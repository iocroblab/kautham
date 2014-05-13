########################################################################
# Cmake module for finding PugiXML
#
# The following variabled will be defined:
#
#  PUGI_XML_FOUND
#  PUGI_XML_INCLUDE_DIRS
#  PUGI_XML_LIBRARY
#

#PUGI_XML
find_path(PUGI_XML_INCLUDE_DIRS pugixml.hpp
   PATHS /usr/include
         /user/local/include)
if(PUGI_XML_INCLUDE_DIRS)
   message("Found PUGI_XML_INCLUDE_DIRS  on: ${PUGI_XML_INCLUDE_DIRS}")
   include_directories(${PUGI_XML_INCLUDE_DIRS})

   find_library (PUGI_XML_LIBRARIES
                NAMES pugixml libpugixml
                PATHS /usr/lib
                      /usr/local/lib
                      /user/local/lib)
   if(PUGI_XML_LIBRARIES)
      message("Found PUGI_XML_LIBRARIES on:" ${PUGI_XML_LIBRARIES})
      set(PUGI_XML_FOUND true)
   else(PUGI_XML_LIBRARIES)
      message(SEND_ERROR "Not found PUGI_XML_LIBRARIES")
   endif(PUGI_XML_LIBRARIES)
else(PUGI_XML_INCLUDE_DIRS)
   message(SEND_ERROR "Not found PUGI_XML_INCLUDE_DIRS")
endif(PUGI_XML_INCLUDE_DIRS)

