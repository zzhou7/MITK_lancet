if(UNIX AND NOT APPLE)
  # See T26955.
  set(MODULE_CUSTOM_TESTS
    mitkPythonConsoleTest.cpp
  )
else()
  set(MODULE_TESTS
    mitkPythonConsoleTest.cpp
  )
endif()
