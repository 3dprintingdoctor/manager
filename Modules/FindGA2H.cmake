 
 find_path (GA2H_DIRS pointclouds.h
      ${CMAKE_INCLUDE_PATH}
      ${CMAKE_INCLUDE_PATH}/includes
      ${CMAKE_CURRENT_SOURCE_DIR}/../../../Aplicaciones/GA2H.git/include
      ${CMAKE_CURRENT_SOURCE_DIR}/../../../Aplicaciones/GA2H_tools/IKUR5Allegro/include
      )
      
 set(GA2H_DIRS ${GA2H_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/../../../Aplicaciones/GA2H_tools/IKUR5Allegro/include)
 set(GA2H_DIRS ${GA2H_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/../../../catkin_ws/src/Kautham.git/src/applications/console)
 set(GA2H_DIRS ${GA2H_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/../../../Aplicaciones/GA2H_tools/Moveup/main)
 set(GA2H_DIRS ${GA2H_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/../../../catkin_ws/src/Kautham.git/src/util/libmt)

if(GA2H_DIRS)
      message(STATUS "Looking for GA2H headers -- found " ${GA2H_DIRS})
      set( GA2H_LIBRARY_PATH ${GA2H_DIRS}/../build )
else(GA2H_DIRS)
      message(SEND_ERROR 
            "Looking for GA2H headers -- not found\n" 
            "Please intall GA2H Library" ${CMAKE_INCLUDE_PATH} )
endif(GA2H_DIRS)
  
find_library (GA2H_LIBRARY 
      NAMES GA2Hands Ga2hands libGA2Hands
      PATHS
      ${GA2H_DIRS}/../build)

if(GA2H_DIRS AND GA2H_LIBRARY)
  set(GA2H_FOUND TRUE)
endif(GA2H_DIRS AND GA2H_LIBRARY)
  
if(GA2H_FOUND)
  if(NOT GA2H_FIND_QUIETLY)
    message(STATUS "Found GA2H: ${GA2H_LIBRARY}")
  endif(NOT GA2H_FIND_QUIETLY)
  else(GA2H_FOUND)
    message(FATAL_ERROR "Could not find GA2H")
endif(GA2H_FOUND)

mark_as_advanced(
    GA2H_LIBRARY_FOUND
    GA2H_INCLUDE_DIR
    GA2H_LIBRARY
)
