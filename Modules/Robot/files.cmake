file(GLOB_RECURSE H_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/include/*")
set(MOC_H_FILES
  include/robotapi.h
  include/robotcontroler.h
  include/robotsocket.h
)
set(CPP_FILES
  robotapi.cpp
  robotcontroler.cpp
  robotsocket.cpp
)

#[[set(RESOURCE_FILES
)]]