file(GLOB_RECURSE H_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/include/*")
set(MOC_H_FILES
  include/gapassessment.h
)
set(CPP_FILES
  gapassessment.cpp
)

#[[set(RESOURCE_FILES
)]]