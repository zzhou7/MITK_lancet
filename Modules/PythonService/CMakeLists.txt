mitkFunctionCheckCompilerFlags("-Wno-cpp" CMAKE_CXX_FLAGS)

mitk_create_module(PythonService
  DEPENDS MitkPython
  AUTOLOAD_WITH MitkPython
)

if(TARGET ${MODULE_TARGET})
  target_link_libraries(${MODULE_TARGET} PUBLIC Python3::NumPy)
  configure_file(PythonPath.h.in "${CMAKE_CURRENT_BINARY_DIR}/PythonPath.h" @ONLY)
endif()

